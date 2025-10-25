"""CLI helper to generate an SVG illustration from a text prompt."""
from __future__ import annotations

import argparse
import logging
import re
from pathlib import Path
from typing import Optional

from examples.marc.vectorize.generate import generate_image
from examples.marc.vectorize.potrace_wrap import trace_bitmap_to_svg
from examples.marc.vectorize.simplify_svg import simplify_svg_file

LOGGER = logging.getLogger(__name__)


def _slugify(text: str, fallback: str = "prompt") -> str:
    slug = re.sub(r"[^a-zA-Z0-9]+", "-", text).strip("-").lower()
    return slug or fallback


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prompt", help="Text prompt describing the desired scene")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("examples/marc/out"),
        help="Directory where intermediate and final artefacts will be written",
    )
    parser.add_argument("--negative-prompt", default=None)
    parser.add_argument("--model", dest="model_id", default=None)
    parser.add_argument("--fallback-model", dest="fallback_model_id", default="runwayml/stable-diffusion-v1-5")
    parser.add_argument("--steps", dest="num_inference_steps", type=int, default=30)
    parser.add_argument("--scale", dest="guidance_scale", type=float, default=7.5)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--threshold", type=float, default=128.0, help="Binarisation threshold for Potrace")
    parser.add_argument("--turdsize", type=int, default=2, help="Potrace turdsize parameter")
    parser.add_argument("--opt-tolerance", type=float, default=0.2, help="Potrace opttolerance parameter")
    parser.add_argument("--page-width", type=int, default=None, help="Desired SVG width")
    parser.add_argument("--page-height", type=int, default=None, help="Desired SVG height")
    parser.add_argument("--simplify-tolerance", type=float, default=2.0, help="Tolerance for SVG simplification")
    parser.add_argument("--no-simplify", action="store_true", help="Skip SVG simplification step")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO)

    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    slug = _slugify(args.prompt)
    png_path = output_dir / f"{slug}.png"
    svg_path = output_dir / f"{slug}.svg"
    simplified_svg_path = output_dir / f"{slug}-simplified.svg"

    LOGGER.info("Generating raster image for prompt '%s'", args.prompt)
    generate_image(
        prompt=args.prompt,
        output_path=png_path,
        model_id=args.model_id,
        fallback_model_id=args.fallback_model_id,
        negative_prompt=args.negative_prompt,
        num_inference_steps=args.num_inference_steps,
        guidance_scale=args.guidance_scale,
        height=args.height,
        width=args.width,
        seed=args.seed,
    )

    page_size = None
    if args.page_width and args.page_height:
        page_size = (args.page_width, args.page_height)

    LOGGER.info("Vectorising PNG with Potrace")
    trace_bitmap_to_svg(
        png_path,
        svg_path,
        threshold=args.threshold,
        turdsize=args.turdsize,
        opt_tolerance=args.opt_tolerance,
        page_size=page_size,
    )

    if not args.no_simplify:
        LOGGER.info("Simplifying SVG geometry")
        simplify_svg_file(svg_path, simplified_svg_path, tolerance=args.simplify_tolerance)
    else:
        LOGGER.info("Skipping SVG simplification step")

    LOGGER.info("PNG written to %s", png_path)
    LOGGER.info("SVG written to %s", svg_path)
    if not args.no_simplify:
        LOGGER.info("Simplified SVG written to %s", simplified_svg_path)

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
