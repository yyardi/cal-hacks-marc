#!/usr/bin/env python
"""High-level drawing helpers for the MARC paint bot demo."""

from __future__ import annotations

import argparse
import json
import logging
import math
from pathlib import Path
from typing import Iterable, Sequence

from examples.marc.calib.page_to_robot import (
    OUTPUT_PATH as DEFAULT_CALIBRATION_PATH,
    stage_default_transform,
)
from examples.marc.constants import (
    DEFAULT_STAGE_DRAW_SPEED,
    DEFAULT_STAGE_PICK_SPEED,
    DEFAULT_STAGE_TRAVEL_SPEED,
    DEFAULT_STAGE_Z_CONTACT,
    DEFAULT_STAGE_Z_SAFE,
    SAFE_WORKSPACE_SIZE_M,
)
from examples.marc.executor import (
    ExecutorConfig,
    MarkerSlot,
    SO100Driver,
    Stroke,
    expand_strokes,
    pick_marker,
    return_marker,
)
from examples.marc.fetch_so101_urdf import download_so101_package
from examples.marc.planner.make_plan import build_plan
from examples.marc.planner.vector_planning import plan_svg_vectors
from examples.marc.run_svg import _slugify
from examples.marc.vectorize.generate import generate_image
from examples.marc.vectorize.potrace_wrap import trace_bitmap_to_svg
from examples.marc.vectorize.simplify_svg import simplify_svg_file

LOGGER = logging.getLogger(__name__)

DEFAULT_URDF_DIR = Path("examples/marc/SO101")
DEFAULT_URDF_PATH = DEFAULT_URDF_DIR / "so101_new_calib.urdf"


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
    raise ValueError(
        f"Unsupported unit '{unit}'. Use one of mm, cm, in, or px."
    )


def _load_marker_config(path: Path) -> dict[str, MarkerSlot]:
    data = json.loads(path.read_text(encoding="utf-8"))
    slots: dict[str, MarkerSlot] = {}
    for color, payload in data.items():
        pick = payload.get("pick") or payload.get("pick_xy")
        if pick is None:
            raise KeyError(
                f"Marker config entry for '{color}' missing 'pick' coordinates"
            )
        return_xy = payload.get("return") or payload.get("return_xy")
        pickup_contact_z = payload.get("pickup_contact_z")
        slots[color] = MarkerSlot(
            color=color,
            pick_xy=tuple(float(v) for v in pick),
            return_xy=(
                tuple(float(v) for v in return_xy)
                if return_xy is not None
                else None
            ),
            pickup_contact_z=(
                float(pickup_contact_z)
                if pickup_contact_z is not None
                else None
            ),
        )
    return slots


def _resolve_urdf_path(urdf_arg: Path | None) -> Path:
    if urdf_arg is not None:
        return urdf_arg.expanduser()

    if DEFAULT_URDF_PATH.exists():
        return DEFAULT_URDF_PATH

    LOGGER.info(
        "URDF path not provided. Downloading default SO101 assets to %s",
        DEFAULT_URDF_DIR,
    )
    try:
        download_so101_package(DEFAULT_URDF_DIR, overwrite=False)
    except FileExistsError:
        # Another process may have started downloading simultaneously. Carry on and
        # rely on the existence check below.
        pass
    except Exception as exc:  # pragma: no cover - networking / filesystem errors
        raise SystemExit(f"Failed to download default URDF: {exc}")

    if not DEFAULT_URDF_PATH.exists():
        raise SystemExit(
            "Unable to locate the default SO101 URDF after download. "
            "Pass --urdf to provide a custom path."
        )

    return DEFAULT_URDF_PATH


def _create_driver(args: argparse.Namespace, page_size: Sequence[float]) -> SO100Driver:
    urdf_path = _resolve_urdf_path(args.urdf)
    executor_cfg = ExecutorConfig(
        travel_speed=args.travel_speed,
        draw_speed=args.draw_speed,
        pick_speed=args.pick_speed,
        z_contact=args.z_contact,
        z_safe=args.z_safe,
        command_rate_hz=args.command_rate,
        pitch_deg=args.pitch,
        roll_deg=args.roll,
        yaw_deg=args.yaw,
    )
    if args.marker_config:
        executor_cfg.marker_slots = _load_marker_config(args.marker_config.expanduser())

    driver_kwargs: dict[str, object]
    if args.use_stage_default:
        transform = stage_default_transform(
            (
                float(page_size[0]) * 1000.0,
                float(page_size[1]) * 1000.0,
            )
        )
        driver_kwargs = {"page_to_robot_matrix": transform.matrix}
        LOGGER.info("Using baked stage calibration transform")
    else:
        homography_path = (
            args.homography.expanduser()
            if args.homography is not None
            else DEFAULT_CALIBRATION_PATH
        )
        if not homography_path.exists():
            raise SystemExit(
                "Calibration file not found at %s. Run `python -m examples.marc.calib.page_to_robot` "
                "to generate it or re-run this command with --use-stage-default to rely on the baked "
                "Cal Hacks transform."
                % homography_path
            )
        driver_kwargs = {"homography_path": homography_path}

    camera_homography = (
        args.camera_homography.expanduser() if args.camera_homography else None
    )
    calibration_dir = (
        args.follower_calibration_dir.expanduser()
        if args.follower_calibration_dir
        else None
    )

    driver = SO100Driver(
        port=args.port,
        urdf_path=urdf_path,
        camera_homography_path=camera_homography,
        page_size=page_size,
        executor_cfg=executor_cfg,
        base_pose=tuple(args.base_pose),
        follower_id=args.follower_id,
        calibration_dir=calibration_dir,
        **driver_kwargs,
    )
    return driver


def _execute_color_sequences(
    args: argparse.Namespace,
    page_size: Sequence[float],
    color_strokes: Sequence[tuple[str, Sequence[Stroke]]],
) -> None:
    if not color_strokes:
        LOGGER.warning("No strokes to execute")
        return

    driver = _create_driver(args, page_size)

    with driver.session(calibrate=args.calibrate) as session:
        for color, strokes in color_strokes:
            LOGGER.info(
                "Executing %d stroke(s) for color '%s'", len(strokes), color or "default"
            )
            commands = []
            if color and color in session.cfg.marker_slots:
                commands.append(pick_marker(color))
            elif color:
                LOGGER.debug(
                    "No marker slot configured for %s; drawing without pick/return", color
                )
            commands.extend(expand_strokes(strokes))
            if color and color in session.cfg.marker_slots:
                commands.append(return_marker(color))
            session.execute(commands)


def _plan_svg(
    svg_path: Path,
    *,
    unit_scale: float,
    page_width_m: float,
    page_height_m: float,
    margin_m: float,
    step: float,
    min_samples: int,
    merge_distance: float,
    pen_up_threshold: float,
) -> tuple[tuple[float, float], list[tuple[str, list[Stroke]]], dict]:
    strokes = plan_svg_vectors(
        svg_path,
        step=step,
        min_samples=min_samples,
        merge_distance=merge_distance,
        pen_up_threshold=pen_up_threshold,
    )
    page_width_units = page_width_m / unit_scale
    page_height_units = page_height_m / unit_scale
    margin_units = margin_m / unit_scale
    plan = build_plan(
        strokes,
        page_width=page_width_units,
        page_height=page_height_units,
        scale=unit_scale,
        margin=margin_units,
    )
    page = plan.get("page", {})
    page_size = (
        float(page.get("width", page_width_m)),
        float(page.get("height", page_height_m)),
    )
    color_entries = []
    for entry in plan.get("colors", []):
        name = str(entry.get("name") or "default")
        stroke_objs: list[Stroke] = []
        for raw_stroke in entry.get("strokes", []):
            if len(raw_stroke) < 2:
                continue
            stroke_objs.append(
                Stroke(points=[[float(x), float(y)] for x, y in raw_stroke])
            )
        if stroke_objs:
            color_entries.append((name, stroke_objs))
    metadata = plan.get("metadata", {})
    return page_size, color_entries, metadata


def _build_star_points(
    page_size: Sequence[float],
    *,
    size_ratio: float,
    inner_ratio: float,
    rotation_deg: float,
) -> list[list[float]]:
    width, height = page_size
    radius = 0.5 * min(width, height) * size_ratio
    inner_radius = radius * inner_ratio
    cx = width / 2.0
    cy = height / 2.0
    points: list[list[float]] = []
    for i in range(5):
        outer_angle = math.radians(rotation_deg + i * 72.0)
        inner_angle = math.radians(rotation_deg + i * 72.0 + 36.0)
        ox = cx + radius * math.cos(outer_angle)
        oy = cy + radius * math.sin(outer_angle)
        ix = cx + inner_radius * math.cos(inner_angle)
        iy = cy + inner_radius * math.sin(inner_angle)
        points.append([ox, oy])
        points.append([ix, iy])
    points.append(points[0])
    return points


def _cmd_star(args: argparse.Namespace) -> int:
    page_size = (args.page_width, args.page_height)
    star_points = _build_star_points(
        page_size,
        size_ratio=args.size,
        inner_ratio=args.inner_ratio,
        rotation_deg=args.rotation,
    )
    strokes = [(args.star_color, [Stroke(points=star_points)])]
    _execute_color_sequences(args, page_size, strokes)
    return 0


def _cmd_svg(args: argparse.Namespace) -> int:
    svg_path = args.svg.expanduser()
    if not svg_path.exists():
        raise SystemExit(f"SVG file not found: {svg_path}")
    scale = _unit_scale(args.unit, args.dpi)
    page_size, color_strokes, metadata = _plan_svg(
        svg_path,
        unit_scale=scale,
        page_width_m=args.page_width,
        page_height_m=args.page_height,
        margin_m=args.margin,
        step=args.step,
        min_samples=args.min_samples,
        merge_distance=args.merge_distance,
        pen_up_threshold=args.pen_up_threshold,
    )
    inside_ratio = float(metadata.get("geometry", {}).get("inside_ratio", 1.0))
    LOGGER.info(
        "Sampling coverage: %.1f%% of %s points within page bounds",
        inside_ratio * 100.0,
        metadata.get("geometry", {}).get("point_count", "0"),
    )
    _execute_color_sequences(args, page_size, color_strokes)
    return 0


def _cmd_prompt(args: argparse.Namespace) -> int:
    output_dir: Path = args.output_dir.expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    slug = args.slug or _slugify(args.prompt)
    png_path = output_dir / f"{slug}.png"
    svg_path = output_dir / f"{slug}.svg"
    simplified_svg_path = output_dir / f"{slug}-simplified.svg"

    if not (args.reuse_intermediates and png_path.exists()):
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
    else:
        LOGGER.info("Reusing existing image at %s", png_path)

    if not (args.reuse_intermediates and svg_path.exists()):
        LOGGER.info("Vectorising diffusion output with Potrace")
        trace_bitmap_to_svg(
            png_path,
            svg_path,
            threshold=args.threshold,
            turdsize=args.turdsize,
            opt_tolerance=args.opt_tolerance,
        )
    else:
        LOGGER.info("Reusing existing SVG at %s", svg_path)

    working_svg = svg_path
    if args.no_simplify:
        LOGGER.info("Skipping SVG simplification step")
    else:
        if not (args.reuse_intermediates and simplified_svg_path.exists()):
            LOGGER.info("Simplifying SVG geometry")
            simplify_svg_file(
                svg_path,
                simplified_svg_path,
                tolerance=args.simplify_tolerance,
            )
        else:
            LOGGER.info("Reusing simplified SVG at %s", simplified_svg_path)
        working_svg = simplified_svg_path

    LOGGER.info("Planning strokes for %s", working_svg)
    scale = _unit_scale(args.unit, args.dpi)
    page_size, color_strokes, metadata = _plan_svg(
        working_svg,
        unit_scale=scale,
        page_width_m=args.page_width,
        page_height_m=args.page_height,
        margin_m=args.margin,
        step=args.step,
        min_samples=args.min_samples,
        merge_distance=args.merge_distance,
        pen_up_threshold=args.pen_up_threshold,
    )
    inside_ratio = float(metadata.get("geometry", {}).get("inside_ratio", 1.0))
    LOGGER.info(
        "Sampling coverage: %.1f%% of %s points within page bounds",
        inside_ratio * 100.0,
        metadata.get("geometry", {}).get("point_count", "0"),
    )
    _execute_color_sequences(args, page_size, color_strokes)
    return 0


def _add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="Serial port of the SO100 follower")
    parser.add_argument(
        "--urdf",
        type=Path,
        help=(
            "Path to the SO100/SO101 follower URDF. If omitted, the default "
            "SO101 package will be downloaded automatically."
        ),
    )
    parser.add_argument(
        "--page-width",
        type=float,
        default=SAFE_WORKSPACE_SIZE_M[0],
        help="Drawing page width in metres",
    )
    parser.add_argument(
        "--page-height",
        type=float,
        default=SAFE_WORKSPACE_SIZE_M[1],
        help="Drawing page height in metres",
    )
    parser.add_argument(
        "--travel-speed",
        type=float,
        default=DEFAULT_STAGE_TRAVEL_SPEED,
        help="Travel speed in metres per second",
    )
    parser.add_argument(
        "--draw-speed",
        type=float,
        default=DEFAULT_STAGE_DRAW_SPEED,
        help="Drawing speed in metres per second",
    )
    parser.add_argument(
        "--pick-speed",
        type=float,
        default=DEFAULT_STAGE_PICK_SPEED,
        help="Marker pick speed in metres per second",
    )
    parser.add_argument(
        "--z-contact",
        type=float,
        default=DEFAULT_STAGE_Z_CONTACT,
        help="Z height when the pen touches the page",
    )
    parser.add_argument(
        "--z-safe",
        type=float,
        default=DEFAULT_STAGE_Z_SAFE,
        help="Safe Z height for travel",
    )
    parser.add_argument("--pitch", type=float, default=-90.0, help="Wrist pitch angle in degrees")
    parser.add_argument("--roll", type=float, default=0.0, help="Wrist roll angle in degrees")
    parser.add_argument("--yaw", type=float, default=180.0, help="Wrist yaw angle in degrees")
    parser.add_argument(
        "--command-rate",
        type=float,
        default=15.0,
        help="Command streaming rate in Hz",
    )
    parser.add_argument(
        "--base-pose",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="XYZ offset applied to all robot targets",
    )
    parser.add_argument("--homography", type=Path, help="Page-to-robot homography file")
    parser.add_argument(
        "--use-stage-default",
        action="store_true",
        help="Use the baked Cal Hacks stage transform instead of a homography file",
    )
    parser.add_argument(
        "--marker-config",
        type=Path,
        help="Optional JSON describing marker pick/return slots",
    )
    parser.add_argument(
        "--camera-homography",
        type=Path,
        help="Optional homography mapping camera pixels to page millimetres",
    )
    parser.add_argument(
        "--follower-id",
        default="marc_so101",
        help="Identifier used when loading/saving follower calibration JSON",
    )
    parser.add_argument(
        "--follower-calibration-dir",
        type=Path,
        help="Directory containing follower calibration JSON files",
    )
    parser.add_argument("--calibrate", action="store_true", help="Run calibration on connect")


def _add_planner_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--unit", choices=["mm", "cm", "in", "px"], default="mm")
    parser.add_argument("--dpi", type=float, default=96.0)
    parser.add_argument(
        "--margin",
        type=float,
        default=0.005,
        help="Margin (in metres) reserved around the drawing",
    )
    parser.add_argument("--step", type=float, default=2.0, help="Sampling step size in SVG units")
    parser.add_argument(
        "--min-samples",
        type=int,
        default=2,
        help="Minimum number of samples per SVG path",
    )
    parser.add_argument(
        "--merge-distance",
        type=float,
        default=18.0,
        help="RGB distance threshold for palette merging",
    )
    parser.add_argument(
        "--pen-up-threshold",
        type=float,
        default=10.0,
        help="Distance threshold for inserting pen-up moves",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="Logging verbosity (e.g. INFO, DEBUG)",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    star = subparsers.add_parser("star", help="Draw a simple five-point star")
    _add_connection_args(star)
    star.add_argument(
        "--size",
        type=float,
        default=0.8,
        help="Outer diameter as a fraction of the shorter page dimension",
    )
    star.add_argument(
        "--inner-ratio",
        type=float,
        default=0.45,
        help="Inner radius as a fraction of the outer radius",
    )
    star.add_argument(
        "--rotation",
        type=float,
        default=-90.0,
        help="Rotation of the star in degrees (0 points to +X)",
    )
    star.add_argument(
        "--star-color",
        default="default",
        help="Label for the star colour (used when matching marker slots)",
    )
    star.set_defaults(func=_cmd_star)

    svg = subparsers.add_parser("butterfly", help="Draw an SVG using the MARC planner")
    _add_connection_args(svg)
    _add_planner_args(svg)
    svg.add_argument("svg", type=Path, help="Path to the SVG file to draw")
    svg.set_defaults(func=_cmd_svg)

    prompt = subparsers.add_parser(
        "diffusion", help="Prompt → diffusion → SVG → draw pipeline"
    )
    _add_connection_args(prompt)
    _add_planner_args(prompt)
    prompt.add_argument("prompt", help="Text prompt describing the drawing")
    prompt.add_argument("--negative-prompt", default=None)
    prompt.add_argument("--model", dest="model_id", default=None)
    prompt.add_argument(
        "--fallback-model",
        dest="fallback_model_id",
        default="runwayml/stable-diffusion-v1-5",
    )
    prompt.add_argument("--steps", dest="num_inference_steps", type=int, default=30)
    prompt.add_argument("--scale", dest="guidance_scale", type=float, default=7.5)
    prompt.add_argument("--height", type=int, default=None)
    prompt.add_argument("--width", type=int, default=None)
    prompt.add_argument("--seed", type=int, default=None)
    prompt.add_argument("--threshold", type=float, default=128.0)
    prompt.add_argument("--turdsize", type=int, default=2)
    prompt.add_argument("--opt-tolerance", type=float, default=0.2)
    prompt.add_argument(
        "--simplify-tolerance", type=float, default=2.0, help="Tolerance for SVG simplification"
    )
    prompt.add_argument("--no-simplify", action="store_true")
    prompt.add_argument(
        "--output-dir",
        type=Path,
        default=Path("examples/marc/out"),
        help="Directory for diffusion artefacts",
    )
    prompt.add_argument("--slug", default=None, help="Optional slug for output files")
    prompt.add_argument(
        "--reuse-intermediates",
        action="store_true",
        help="Reuse existing diffusion artefacts if present",
    )
    prompt.set_defaults(func=_cmd_prompt)

    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(levelname)s: %(message)s",
    )
    try:
        return args.func(args)
    except Exception:  # pragma: no cover - surface errors nicely to the user
        LOGGER.exception("Paint bot command failed")
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
