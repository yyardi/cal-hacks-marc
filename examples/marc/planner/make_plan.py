"""Generate a MARC execution plan JSON from an SVG drawing."""
from __future__ import annotations

import argparse
import json
from collections import OrderedDict
from pathlib import Path
from typing import Mapping, Sequence

from . import OrderedStroke, color_to_hex
from ..constants import SAFE_WORKSPACE_SIZE_MM
from .vector_planning import plan_svg_vectors


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


def build_plan(
    strokes: Sequence[OrderedStroke],
    *,
    page_width: float,
    page_height: float,
    scale: float,
) -> dict:
    grouped = _group_strokes_by_color(strokes)
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
    return {"page": page, "colors": colors}


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("svg", type=Path, help="Input SVG to convert into a robot plan")
    parser.add_argument("--output", type=Path, required=True, help="Destination JSON file")
    parser.add_argument(
        "--page-width",
        type=float,
        default=SAFE_WORKSPACE_SIZE_MM[0],
        help=f"Page width in the specified units (default: {SAFE_WORKSPACE_SIZE_MM[0]:.0f} mm)",
    )
    parser.add_argument(
        "--page-height",
        type=float,
        default=SAFE_WORKSPACE_SIZE_MM[1],
        help=f"Page height in the specified units (default: {SAFE_WORKSPACE_SIZE_MM[1]:.0f} mm)",
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
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(plan, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())


__all__ = ["build_plan", "main"]
