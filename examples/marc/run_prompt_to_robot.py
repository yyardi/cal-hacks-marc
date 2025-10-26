"""One-shot pipeline: prompt → image → SVG → plan → robot draw."""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
from typing import Sequence

from .constants import (
    DEFAULT_STAGE_DRAW_SPEED,
    DEFAULT_STAGE_PICK_SPEED,
    DEFAULT_STAGE_TRAVEL_SPEED,
    DEFAULT_STAGE_Z_CONTACT,
    DEFAULT_STAGE_Z_SAFE,
    SAFE_WORKSPACE_SIZE_MM,
)
from .planner import make_plan as planner_module
from .planner.make_plan import build_plan
from .planner.vector_planning import plan_svg_vectors
from .run_draw_lerobot_ik import (
    _load_plan,
    _parse_args as parse_draw_args,
    _validate_plan_bounds,
    execute_plan,
)
from .run_svg import _slugify
from .vectorize.generate import generate_image
from .vectorize.potrace_wrap import trace_bitmap_to_svg
from .vectorize.simplify_svg import simplify_svg_file

LOGGER = logging.getLogger(__name__)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prompt", help="Text prompt describing the drawing")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("examples/marc/out"),
        help="Directory for all generated artefacts",
    )
    parser.add_argument("--slug", help="Optional slug to reuse instead of deriving from the prompt")
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
    parser.add_argument(
        "--simplify-tolerance",
        type=float,
        default=2.0,
        help="Tolerance used when simplifying the Potrace output",
    )
    parser.add_argument("--no-simplify", action="store_true", help="Skip SVG simplification")

    default_page_width_mm = float(SAFE_WORKSPACE_SIZE_MM[0])
    default_page_height_mm = float(SAFE_WORKSPACE_SIZE_MM[1])
    parser.add_argument(
        "--page-width",
        type=float,
        default=default_page_width_mm,
        help="Target page width expressed in --unit (default: safe workspace width)",
    )
    parser.add_argument(
        "--page-height",
        type=float,
        default=default_page_height_mm,
        help="Target page height expressed in --unit (default: safe workspace height)",
    )
    parser.add_argument(
        "--unit",
        choices=["mm", "cm", "in", "px"],
        default="mm",
        help="Units shared by the SVG coordinates and page size",
    )
    parser.add_argument("--dpi", type=float, default=96.0, help="Pixels per inch when --unit=px")
    parser.add_argument("--margin", type=float, default=5.0, help="Margin (in --unit) around the drawing")
    parser.add_argument("--step", type=float, default=2.0, help="Sampling step size passed to the planner")
    parser.add_argument("--min-samples", type=int, default=2, help="Minimum samples per SVG path")
    parser.add_argument("--merge-distance", type=float, default=18.0, help="RGB distance threshold for palette merging")
    parser.add_argument("--pen-up-threshold", type=float, default=10.0, help="Distance threshold for inserting pen-up moves")

    parser.add_argument("--skip-draw", action="store_true", help="Stop after writing the plan JSON")
    parser.add_argument(
        "--reuse-intermediates",
        action="store_true",
        help="Reuse existing PNG/SVG/plan artefacts for the slug instead of regenerating them",
    )
    parser.add_argument("--port", help="Serial port of the SO100 follower")
    parser.add_argument("--urdf", type=Path, help="Path to the follower URDF file")
    parser.add_argument("--homography", type=Path, help="Page-to-robot homography file")
    parser.add_argument("--use-stage-default", action="store_true", help="Use the baked stage calibration")
    parser.add_argument("--camera-homography", type=Path, help="Optional camera correction homography")
    parser.add_argument("--marker-config", type=Path, help="Optional JSON describing marker pick/return slots")
    parser.add_argument("--base-pose", type=float, nargs=3, default=(0.0, 0.0, 0.0), help="XYZ offset of the robot base")
    parser.add_argument("--calibrate", action="store_true", help="Run calibration when connecting to the arm")
    parser.add_argument("--correct", action="store_true", help="Enable vision correction after each colour")
    parser.add_argument("--target-image", type=Path, help="Optional raster used for correction")
    parser.add_argument("--correction-threshold", type=float, default=0.2)
    parser.add_argument("--correction-min-component", type=int, default=12)
    parser.add_argument(
        "--correction-resolution",
        type=int,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        default=(1024, 1024),
        help="Resolution used when warping imagery for the correction pass",
    )

    parser.add_argument("--travel-speed", type=float, default=DEFAULT_STAGE_TRAVEL_SPEED)
    parser.add_argument("--draw-speed", type=float, default=DEFAULT_STAGE_DRAW_SPEED)
    parser.add_argument("--pick-speed", type=float, default=DEFAULT_STAGE_PICK_SPEED)
    parser.add_argument("--z-contact", type=float, default=DEFAULT_STAGE_Z_CONTACT)
    parser.add_argument("--z-safe", type=float, default=DEFAULT_STAGE_Z_SAFE)
    parser.add_argument("--pitch", type=float, default=-90.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=180.0)
    parser.add_argument("--command-rate", type=float, default=15.0)
    parser.add_argument("--log-level", default="INFO")
    return parser


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _build_draw_argv(args: argparse.Namespace, plan_path: Path, page_width_m: float, page_height_m: float) -> list[str]:
    argv = [
        "--plan",
        str(plan_path),
        "--port",
        args.port,
        "--urdf",
        str(args.urdf),
        "--page-width",
        f"{page_width_m}",
        "--page-height",
        f"{page_height_m}",
        "--travel-speed",
        f"{args.travel_speed}",
        "--draw-speed",
        f"{args.draw_speed}",
        "--pick-speed",
        f"{args.pick_speed}",
        "--z-contact",
        f"{args.z_contact}",
        "--z-safe",
        f"{args.z_safe}",
        "--pitch",
        f"{args.pitch}",
        "--roll",
        f"{args.roll}",
        "--yaw",
        f"{args.yaw}",
        "--command-rate",
        f"{args.command_rate}",
        "--log-level",
        args.log_level,
    ]

    if args.marker_config:
        argv.extend(["--marker-config", str(args.marker_config)])
    if args.camera_homography:
        argv.extend(["--camera-homography", str(args.camera_homography)])
    if args.homography:
        argv.extend(["--homography", str(args.homography)])
    if args.use_stage_default:
        argv.append("--use-stage-default")
    if args.calibrate:
        argv.append("--calibrate")
    if args.correct:
        argv.append("--correct")
    if args.target_image:
        argv.extend(["--target-image", str(args.target_image)])
    if args.base_pose is not None:
        argv.extend(["--base-pose", *(f"{value}" for value in args.base_pose)])
    argv.extend(["--correction-threshold", f"{args.correction_threshold}"])
    argv.extend(["--correction-min-component", f"{args.correction_min_component}"])
    argv.extend(["--correction-resolution", *(str(v) for v in args.correction_resolution)])
    return argv


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    slug = args.slug or _slugify(args.prompt)
    png_path = output_dir / f"{slug}.png"
    svg_path = output_dir / f"{slug}.svg"
    simplified_svg_path = output_dir / f"{slug}-simplified.svg"
    plan_path = output_dir / f"{slug}_plan.json"

    unit_scale = planner_module._unit_scale(args.unit, args.dpi)

    plan: dict | None = None
    if args.reuse_intermediates and plan_path.exists():
        LOGGER.info("Reusing existing plan at %s", plan_path)
        plan = json.loads(plan_path.read_text(encoding="utf-8"))

    if plan is None:
        if args.reuse_intermediates and png_path.exists():
            LOGGER.info("Reusing existing raster at %s", png_path)
        else:
            LOGGER.info("Generating raster for prompt '%s'", args.prompt)
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
            LOGGER.info("Raster written to %s", png_path)

    page_size = None
    if args.page_width and args.page_height:
        page_size = (float(args.page_width), float(args.page_height))

    svg_for_planning = svg_path
    if plan is None:
        if args.reuse_intermediates and svg_path.exists():
            LOGGER.info("Reusing existing SVG at %s", svg_path)
        else:
            LOGGER.info("Vectorising raster with Potrace")
            trace_bitmap_to_svg(
                png_path,
                svg_path,
                threshold=args.threshold,
                turdsize=args.turdsize,
                opt_tolerance=args.opt_tolerance,
                page_size=page_size,
            )
            LOGGER.info("SVG written to %s", svg_path)

        if not args.no_simplify:
            if args.reuse_intermediates and simplified_svg_path.exists():
                LOGGER.info("Reusing simplified SVG at %s", simplified_svg_path)
            else:
                LOGGER.info("Simplifying SVG geometry")
                simplify_svg_file(svg_path, simplified_svg_path, tolerance=args.simplify_tolerance)
                LOGGER.info("Simplified SVG written to %s", simplified_svg_path)
            svg_for_planning = simplified_svg_path

        LOGGER.info("Sampling SVG paths into strokes")
        strokes = plan_svg_vectors(
            svg_for_planning,
            step=args.step,
            min_samples=args.min_samples,
            merge_distance=args.merge_distance,
            pen_up_threshold=args.pen_up_threshold,
        )

        LOGGER.info(
            "Fitting drawing to %.2f×%.2f %s page with %.2f %s margin",
            args.page_width,
            args.page_height,
            args.unit,
            args.margin,
            args.unit,
        )
        plan = build_plan(
            strokes,
            page_width=args.page_width,
            page_height=args.page_height,
            scale=unit_scale,
            margin=args.margin,
        )
        _write_json(plan_path, plan)
        LOGGER.info("Plan written to %s", plan_path)

    if plan is None:
        plan = json.loads(plan_path.read_text(encoding="utf-8"))

    metadata = plan.get("metadata", {})
    geometry = metadata.get("geometry", {})
    inside_ratio = float(geometry.get("inside_ratio", 0.0))
    LOGGER.info(
        "Sampling coverage: %.1f%% of %s points within page bounds",
        inside_ratio * 100.0,
        geometry.get("point_count", "0"),
    )
    if inside_ratio < 0.9:
        LOGGER.warning(
            "Coverage below 90%%. Consider reducing --margin or scaling the SVG further before drawing."
        )

    page_width_m = float(args.page_width) * unit_scale
    page_height_m = float(args.page_height) * unit_scale
    if args.use_stage_default and args.homography:
        LOGGER.warning("--use-stage-default overrides the supplied --homography path")

    plan_size, color_plans = _load_plan(plan_path)
    _validate_plan_bounds(color_plans, page_width_m, page_height_m)
    LOGGER.info(
        "Validated plan bounds: %.3f×%.3f m page, %d colours",
        plan_size[0],
        plan_size[1],
        len(color_plans),
    )

    if args.skip_draw:
        LOGGER.info("Skipping draw step (--skip-draw supplied)")
        return 0

    if not args.port or not args.urdf:
        parser.error("--port and --urdf are required unless --skip-draw is set")

    draw_argv = _build_draw_argv(args, plan_path, page_width_m, page_height_m)
    draw_args = parse_draw_args(draw_argv)
    execute_plan(draw_args)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
