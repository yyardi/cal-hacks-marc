"""High-level vector planning API for MARC plotting."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Mapping, Sequence

from . import OrderedStroke
from .color_quant import MARKER_PALETTE, quantize_paths
from .order_paths import order_paths_by_colour
from .sample_curves import sample_svg_paths


def plan_svg_vectors(
    svg_file: str | Path,
    *,
    step: float = 2.0,
    min_samples: int = 2,
    palette: Mapping[str, tuple[int, int, int]] | None = None,
    merge_distance: float = 18.0,
    pen_up_threshold: float = 10.0,
) -> List[OrderedStroke]:
    """Plan ordered stroke commands from an SVG file."""

    sampled_paths = sample_svg_paths(str(svg_file), step=step, min_samples=min_samples)
    grouped = quantize_paths(
        sampled_paths,
        palette=palette if palette is not None else MARKER_PALETTE,
        merge_distance=merge_distance,
    )
    ordered = order_paths_by_colour(grouped, pen_up_threshold=pen_up_threshold)
    return ordered


def strokes_to_json_serialisable(strokes: Sequence[OrderedStroke]) -> List[dict]:
    return [stroke.to_dict() for stroke in strokes]


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plan MARC plotting commands from an SVG.")
    parser.add_argument("svg", help="Path to the input SVG file")
    parser.add_argument("--step", type=float, default=2.0, help="Sampling step size in SVG units")
    parser.add_argument(
        "--min-samples",
        type=int,
        default=2,
        help="Minimum number of points sampled per SVG path",
    )
    parser.add_argument(
        "--merge-distance",
        type=float,
        default=18.0,
        help="Maximum RGB distance for merging near-identical hues",
    )
    parser.add_argument(
        "--pen-up-threshold",
        type=float,
        default=10.0,
        help="Distance threshold for inserting pen-up moves",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Destination JSON file for the ordered strokes (prints to stdout if omitted)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    strokes = plan_svg_vectors(
        args.svg,
        step=args.step,
        min_samples=args.min_samples,
        merge_distance=args.merge_distance,
        pen_up_threshold=args.pen_up_threshold,
        palette=MARKER_PALETTE,
    )
    payload = strokes_to_json_serialisable(strokes)

    output_path: Path | None = args.output
    if output_path:
        output_path.write_text(json.dumps(payload, indent=2))
    else:
        print(json.dumps(payload, indent=2))

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
