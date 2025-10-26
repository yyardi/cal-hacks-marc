"""Generate a MARC execution plan JSON from an SVG drawing."""
from __future__ import annotations

import argparse
import json
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence, Tuple

import numpy as np

from . import OrderedStroke, color_to_hex
from ..constants import SAFE_WORKSPACE_SIZE_MM
from .vector_planning import plan_svg_vectors


@dataclass(frozen=True)
class PlanGeometry:
    """Geometry metadata describing how an SVG was normalised onto the page."""

    source_bounds: Tuple[float, float, float, float]
    target_bounds: Tuple[float, float, float, float]
    scale_factor: float
    margin: float
    inside_ratio: float
    point_count: int

    def to_dict(self) -> dict[str, object]:
        return {
            "source_bounds": {
                "min_x": float(self.source_bounds[0]),
                "min_y": float(self.source_bounds[1]),
                "max_x": float(self.source_bounds[2]),
                "max_y": float(self.source_bounds[3]),
            },
            "target_bounds": {
                "min_x": float(self.target_bounds[0]),
                "min_y": float(self.target_bounds[1]),
                "max_x": float(self.target_bounds[2]),
                "max_y": float(self.target_bounds[3]),
            },
            "scale_factor": float(self.scale_factor),
            "margin": float(self.margin),
            "inside_ratio": float(self.inside_ratio),
            "point_count": int(self.point_count),
        }


def _unit_scale(unit: str, dpi: float) -> float:
    unit = unit.lower()
    if unit == "mm":
        return 1e-3
    if unit == "cm":
        return 1e-2
    if unit in {"in", "inch", "inches"}:
        return 0.0254
    if unit == "px":
        return 0.0254 / dpi
    raise ValueError(f"Unsupported unit '{unit}'. Use mm, cm, in, or px.")


def _group_strokes_by_color(strokes: Sequence[OrderedStroke]) -> Mapping[str, list[list[list[float]]]]:
    grouped: "OrderedDict[str, list[list[list[float]]]]" = OrderedDict()
    for stroke in strokes:
        attributes = stroke.attributes or {}
        color_name = attributes.get("color_name")
        if not color_name:
            color_name = color_to_hex(tuple(int(c) for c in stroke.color))
        grouped.setdefault(color_name, []).append(stroke.points.tolist())
    return grouped


def _collect_points(strokes: Iterable[OrderedStroke]) -> np.ndarray:
    arrays = [stroke.points for stroke in strokes if stroke.points.size]
    if not arrays:
        raise ValueError("No stroke points were generated from the SVG.")
    return np.vstack(arrays)


def normalize_strokes_to_page(
    strokes: Sequence[OrderedStroke],
    *,
    page_width: float,
    page_height: float,
    margin: float = 0.0,
) -> tuple[list[OrderedStroke], PlanGeometry]:
    """Center and scale the supplied strokes to fit within the page bounds.

    ``page_width`` and ``page_height`` are expressed in the same units as the input
    SVG geometry (typically millimetres or pixels).  ``margin`` reserves a border
    around the drawing region in those units.
    """

    if page_width <= 0 or page_height <= 0:
        raise ValueError("Page dimensions must be positive when normalising strokes")
    if margin < 0:
        raise ValueError("Margin must be non-negative")

    all_points = _collect_points(strokes)
    min_x = float(np.min(all_points[:, 0]))
    max_x = float(np.max(all_points[:, 0]))
    min_y = float(np.min(all_points[:, 1]))
    max_y = float(np.max(all_points[:, 1]))

    source_bounds = (min_x, min_y, max_x, max_y)
    width = max_x - min_x
    height = max_y - min_y

    available_width = page_width - 2 * margin
    available_height = page_height - 2 * margin
    if available_width <= 0 or available_height <= 0:
        raise ValueError("Margin leaves no room for the drawing on the page")

    eps = 1e-9
    scale_candidates = []
    if width > eps:
        scale_candidates.append(available_width / width)
    else:
        scale_candidates.append(float("inf"))
    if height > eps:
        scale_candidates.append(available_height / height)
    else:
        scale_candidates.append(float("inf"))

    scale_factor = min(scale_candidates)
    if not np.isfinite(scale_factor) or scale_factor <= 0:
        scale_factor = 1.0

    scaled_width = width * scale_factor if width > eps else 0.0
    scaled_height = height * scale_factor if height > eps else 0.0

    offset_x = (page_width - scaled_width) / 2.0 if width > eps else page_width / 2.0
    offset_y = (page_height - scaled_height) / 2.0 if height > eps else page_height / 2.0

    normalised: list[OrderedStroke] = []
    inside_total = 0
    total_points = 0
    tolerance = 1e-6

    for stroke in strokes:
        points = stroke.points
        transformed = np.empty_like(points)

        if width > eps:
            transformed[:, 0] = (points[:, 0] - min_x) * scale_factor + offset_x
        else:
            transformed[:, 0] = offset_x

        if height > eps:
            transformed[:, 1] = (points[:, 1] - min_y) * scale_factor + offset_y
        else:
            transformed[:, 1] = offset_y

        mask = (
            (transformed[:, 0] >= -tolerance)
            & (transformed[:, 0] <= page_width + tolerance)
            & (transformed[:, 1] >= -tolerance)
            & (transformed[:, 1] <= page_height + tolerance)
        )
        inside_total += int(np.sum(mask))
        total_points += transformed.shape[0]

        normalised.append(
            OrderedStroke(
                points=transformed,
                color=stroke.color,
                pen_up=stroke.pen_up,
                attributes=dict(stroke.attributes),
            )
        )

    inside_ratio = float(inside_total) / float(total_points) if total_points else 1.0
    target_bounds = (
        offset_x,
        offset_y,
        offset_x + scaled_width,
        offset_y + scaled_height,
    )

    geometry = PlanGeometry(
        source_bounds=source_bounds,
        target_bounds=target_bounds,
        scale_factor=scale_factor,
        margin=margin,
        inside_ratio=inside_ratio,
        point_count=total_points,
    )
    return normalised, geometry


def build_plan(
    strokes: Sequence[OrderedStroke],
    *,
    page_width: float,
    page_height: float,
    scale: float,
    margin: float = 0.0,
) -> dict:
    normalised_strokes, geometry = normalize_strokes_to_page(
        strokes, page_width=page_width, page_height=page_height, margin=margin
    )
    grouped = _group_strokes_by_color(normalised_strokes)
    page = {"width": page_width * scale, "height": page_height * scale}
    colors = [
        {
            "name": name,
            "strokes": [
                [[scale * float(x), scale * float(y)] for x, y in stroke]
                for stroke in stroke_list
            ],
        }
        for name, stroke_list in grouped.items()
    ]
    geometry_dict = geometry.to_dict()
    metadata = {
        "unit_scale": float(scale),
        "margin": float(margin),
        "geometry": geometry_dict,
        "target_bounds_m": {
            axis: float(scale * value)
            for axis, value in geometry_dict["target_bounds"].items()
        },
    }
    return {"page": page, "colors": colors, "metadata": metadata}


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("svg", type=Path, help="Input SVG to convert into a robot plan")
    parser.add_argument("--output", type=Path, required=True, help="Destination JSON file")
    default_page_width_mm = float(SAFE_WORKSPACE_SIZE_MM[0])
    default_page_height_mm = float(SAFE_WORKSPACE_SIZE_MM[1])
    parser.add_argument(
        "--page-width",
        type=float,
        default=default_page_width_mm,
        help=(
            "Page width in the specified units (default: "
            f"{default_page_width_mm:.0f} mm)"
        ),
    )
    parser.add_argument(
        "--page-height",
        type=float,
        default=default_page_height_mm,
        help=(
            "Page height in the specified units (default: "
            f"{default_page_height_mm:.0f} mm)"
        ),
    )
    parser.add_argument(
        "--unit",
        choices=["mm", "cm", "in", "px"],
        default="mm",
        help="Units used by the SVG coordinates and page dimensions",
    )
    parser.add_argument(
        "--dpi",
        type=float,
        default=96.0,
        help="Pixels per inch when --unit=px (ignored otherwise)",
    )
    parser.add_argument("--step", type=float, default=2.0, help="Sampling step size passed to the planner")
    parser.add_argument("--min-samples", type=int, default=2, help="Minimum samples per SVG path")
    parser.add_argument("--merge-distance", type=float, default=18.0, help="RGB distance threshold for palette merging")
    parser.add_argument("--pen-up-threshold", type=float, default=10.0, help="Distance threshold for inserting pen-up moves")
    parser.add_argument(
        "--margin",
        type=float,
        default=5.0,
        help="Margin to keep around the drawing when fitting it to the page (same units as --page-width)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    scale = _unit_scale(args.unit, args.dpi)
    strokes = plan_svg_vectors(
        args.svg,
        step=args.step,
        min_samples=args.min_samples,
        merge_distance=args.merge_distance,
        pen_up_threshold=args.pen_up_threshold,
    )

    plan = build_plan(
        strokes,
        page_width=args.page_width,
        page_height=args.page_height,
        scale=scale,
        margin=args.margin,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(plan, indent=2), encoding="utf-8")
    print(f"✓ Plan written to {args.output}")
    metadata = plan.get("metadata", {})
    geometry = metadata.get("geometry")
    if isinstance(geometry, dict):
        source_bounds = geometry.get("source_bounds", {})
        target_bounds = geometry.get("target_bounds", {})
        inside_ratio = geometry.get("inside_ratio")
        point_count = geometry.get("point_count")
        print(
            "Source bounds (input units): "
            f"[{source_bounds.get('min_x', 0.0):.2f}, {source_bounds.get('min_y', 0.0):.2f}] → "
            f"[{source_bounds.get('max_x', 0.0):.2f}, {source_bounds.get('max_y', 0.0):.2f}]"
        )
        print(
            "Target bounds (page units): "
            f"[{target_bounds.get('min_x', 0.0):.2f}, {target_bounds.get('min_y', 0.0):.2f}] → "
            f"[{target_bounds.get('max_x', 0.0):.2f}, {target_bounds.get('max_y', 0.0):.2f}]"
        )
        if inside_ratio is not None and point_count is not None:
            print(
                f"In-bounds coverage: {inside_ratio * 100:.1f}% of {int(point_count)} sampled points"
            )
        print(f"Margin applied: {metadata.get('margin', args.margin):.2f} {args.unit}")
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())


__all__ = ["PlanGeometry", "build_plan", "main", "normalize_strokes_to_page"]
