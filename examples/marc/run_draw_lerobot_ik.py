"""CLI to execute Lerobot IK drawing plans on the MARC setup."""

from __future__ import annotations

import argparse
import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from .executor import (
    DriverCommand,
    ExecutorConfig,
    MarkerSlot,
    SO100Driver,
    Stroke,
    expand_strokes,
    pick_marker,
    return_marker,
)

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class ColorPlan:
    """Stroke information for a specific marker color."""

    name: str
    strokes: list[Stroke]
    raster_path: Path | None = None


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path, help="Path to the JSON plan file")
    parser.add_argument("--port", required=True, help="Serial port of the SO100 follower")
    parser.add_argument("--urdf", required=True, type=Path, help="Path to the follower URDF file")
    parser.add_argument("--homography", required=True, type=Path, help="Homography mapping page to robot")
    parser.add_argument("--page-width", type=float, required=True, help="Drawing page width in meters")
    parser.add_argument("--page-height", type=float, required=True, help="Drawing page height in meters")
    parser.add_argument("--travel-speed", type=float, default=0.04, help="Travel speed in page units per second")
    parser.add_argument("--draw-speed", type=float, default=0.02, help="Drawing speed in page units per second")
    parser.add_argument("--pick-speed", type=float, default=0.015, help="Speed used when picking markers")
    parser.add_argument("--z-contact", type=float, default=-0.01, help="Z height when the pen touches the page")
    parser.add_argument("--z-safe", type=float, default=0.05, help="Safe Z height for travel")
    parser.add_argument("--pitch", type=float, default=-90.0, help="Pitch angle (degrees) of the wrist")
    parser.add_argument("--roll", type=float, default=0.0, help="Roll angle (degrees) of the wrist")
    parser.add_argument("--yaw", type=float, default=180.0, help="Yaw angle (degrees) of the wrist")
    parser.add_argument("--command-rate", type=float, default=15.0, help="Command streaming rate (Hz)")
    parser.add_argument("--marker-config", type=Path, help="Optional JSON describing marker pick/return slots")
    parser.add_argument(
        "--camera-homography",
        type=Path,
        help="Optional homography mapping camera pixels to page millimetres for correction",
    )
    parser.add_argument("--base-pose", type=float, nargs=3, default=(0.0, 0.0, 0.0), help="XYZ offset of the robot base")
    parser.add_argument("--calibrate", action="store_true", help="Run calibration on connect")
    parser.add_argument("--correct", action="store_true", help="Enable correction pass after each color")
    parser.add_argument("--target-image", type=Path, help="Override path to the target raster image")
    parser.add_argument("--correction-threshold", type=float, default=0.2, help="Residual threshold for correction")
    parser.add_argument("--correction-min-component", type=int, default=12, help="Ignore residual blobs smaller than this")
    parser.add_argument(
        "--correction-resolution",
        type=int,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        default=(1024, 1024),
        help="Resolution to warp correction imagery into",
    )
    parser.add_argument("--log-level", default="INFO", help="Logging verbosity")
    return parser.parse_args()


def _load_plan(path: Path) -> tuple[tuple[float, float], list[ColorPlan]]:
    with path.open("r", encoding="utf-8") as fh:
        data = json.load(fh)

    if "page" not in data:
        raise KeyError("Plan file must contain a 'page' section with 'width' and 'height'")
    page_data = data["page"]
    if isinstance(page_data, dict):
        width = float(page_data.get("width"))
        height = float(page_data.get("height"))
    elif isinstance(page_data, (list, tuple)) and len(page_data) == 2:
        width, height = map(float, page_data)
    else:
        raise ValueError("'page' must be a dict with width/height or a two element list")

    colors: list[ColorPlan] = []
    for entry in data.get("colors", []):
        name = entry.get("name")
        if not name:
            raise KeyError("Each color entry must provide a 'name'")
        raw_strokes: Iterable[Iterable[Iterable[float]]] = entry.get("strokes", [])
        strokes = [Stroke(points=[[float(x), float(y)] for x, y in stroke]) for stroke in raw_strokes]
        raster = entry.get("raster")
        colors.append(ColorPlan(name=name, strokes=strokes, raster_path=Path(raster) if raster else None))
    if not colors:
        raise ValueError("Plan file must define at least one color")
    return (width, height), colors


def _load_marker_config(path: Path) -> dict[str, MarkerSlot]:
    with path.open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    slots: dict[str, MarkerSlot] = {}
    for color, payload in data.items():
        pick = payload.get("pick") or payload.get("pick_xy")
        if pick is None:
            raise KeyError(f"Marker config entry for '{color}' missing 'pick' coordinates")
        return_xy = payload.get("return") or payload.get("return_xy")
        pickup_z = payload.get("pickup_contact_z") or payload.get("z")
        slots[color] = MarkerSlot(
            color=color,
            pick_xy=tuple(float(v) for v in pick),
            return_xy=tuple(float(v) for v in return_xy) if return_xy is not None else None,
            pickup_contact_z=float(pickup_z) if pickup_z is not None else None,
        )
    return slots


def _load_raster(path: Path) -> np.ndarray:
    from PIL import Image

    image = Image.open(path)
    return np.asarray(image.convert("RGB"))


def main() -> None:
    args = _parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    (plan_width, plan_height), color_plans = _load_plan(args.plan)
    if not np.isclose(plan_width, args.page_width) or not np.isclose(plan_height, args.page_height):
        logger.warning(
            "Plan page size (%.3f, %.3f) differs from CLI (%.3f, %.3f)",
            plan_width,
            plan_height,
            args.page_width,
            args.page_height,
        )

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
        executor_cfg.marker_slots = _load_marker_config(args.marker_config)

    driver = SO100Driver(
        port=args.port,
        urdf_path=args.urdf,
        homography_path=args.homography,
        camera_homography_path=args.camera_homography,
        page_size=(args.page_width, args.page_height),
        executor_cfg=executor_cfg,
        base_pose=args.base_pose,
    )

    with driver.session(calibrate=args.calibrate) as session:
        for color_plan in color_plans:
            logger.info("Executing color: %s (%d strokes)", color_plan.name, len(color_plan.strokes))
            commands: list[DriverCommand] = []

            has_marker_slot = color_plan.name in session.cfg.marker_slots
            if has_marker_slot:
                commands.append(pick_marker(color_plan.name))
            else:
                logger.debug(
                    "No marker slot configured for %s; running strokes without pick/return",
                    color_plan.name,
                )

            commands.extend(expand_strokes(color_plan.strokes))

            if has_marker_slot:
                commands.append(return_marker(color_plan.name))
            session.execute(commands)

            if args.correct:
                raster_path = args.target_image or color_plan.raster_path
                if raster_path is None:
                    logger.warning(
                        "Correction requested but no raster provided for %s; skipping correction",
                        color_plan.name,
                    )
                    continue
                target = _load_raster(raster_path)
                resolution = tuple(map(int, args.correction_resolution))
                session.perform_correction(
                    target_raster=target,
                    homography_resolution=resolution,
                    threshold=args.correction_threshold,
                    min_component=args.correction_min_component,
                )


if __name__ == "__main__":
    main()
