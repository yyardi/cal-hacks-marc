"""Quick sanity script to draw an alignment square with the SO101."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Iterable, Sequence

from .constants import (
    DEFAULT_STAGE_DRAW_SPEED,
    DEFAULT_STAGE_MARGIN_MM,
    DEFAULT_STAGE_TRAVEL_SPEED,
    DEFAULT_STAGE_Z_CONTACT,
    DEFAULT_STAGE_Z_SAFE,
    SAFE_WORKSPACE_SIZE_MM,
)
from .calib.page_to_robot import OUTPUT_PATH as DEFAULT_CALIBRATION_PATH
from .calib.page_to_robot import stage_default_transform
from .executor import (
    ExecutorConfig,
    MOVE_TO,
    PEN_DOWN,
    PEN_UP,
    DriverCommand,
    SO100Driver,
)


def _parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port of the SO101 follower")
    parser.add_argument("--urdf", required=True, type=Path, help="Path to the SO101 URDF file")
    parser.add_argument(
        "--page-to-robot",
        type=Path,
        help=(
            "Rigid transform .npy produced by calib.page_to_robot. "
            "If omitted the command looks for examples/marc/out/calib_page_to_robot.npy."
        ),
    )
    parser.add_argument(
        "--use-stage-default",
        action="store_true",
        help=(
            "Use the baked Cal Hacks stage calibration instead of loading a file. "
            "This matches the origin/+X/+Y jog described in the README."
        ),
    )
    default_page_width_mm = float(SAFE_WORKSPACE_SIZE_MM[0])
    default_page_height_mm = float(SAFE_WORKSPACE_SIZE_MM[1])
    parser.add_argument(
        "--page-width-mm",
        type=float,
        default=default_page_width_mm,
        help=(
            "Physical drawing width in millimetres (default: "
            f"{default_page_width_mm:.0f} mm)"
        ),
    )
    parser.add_argument(
        "--page-height-mm",
        type=float,
        default=default_page_height_mm,
        help=(
            "Physical drawing height in millimetres (default: "
            f"{default_page_height_mm:.0f} mm)"
        ),
    )
    parser.add_argument(
        "--square-size-mm",
        type=float,
        default=110.0,
        help="Side length of the test square in millimetres",
    )
    parser.add_argument(
        "--margin-mm",
        type=float,
        default=DEFAULT_STAGE_MARGIN_MM,
        help="Minimum margin to keep from each page edge in millimetres",
    )
    parser.add_argument(
        "--travel-speed",
        type=float,
        default=DEFAULT_STAGE_TRAVEL_SPEED,
        help="Travel speed in m/s",
    )
    parser.add_argument(
        "--draw-speed",
        type=float,
        default=DEFAULT_STAGE_DRAW_SPEED,
        help="Draw speed in m/s",
    )
    parser.add_argument(
        "--z-contact",
        type=float,
        default=DEFAULT_STAGE_Z_CONTACT,
        help="Z contact height in meters",
    )
    parser.add_argument(
        "--z-safe",
        type=float,
        default=DEFAULT_STAGE_Z_SAFE,
        help="Safe travel Z height in meters",
    )
    parser.add_argument("--pitch", type=float, default=-90.0, help="Wrist pitch angle in degrees")
    parser.add_argument("--roll", type=float, default=0.0, help="Wrist roll angle in degrees")
    parser.add_argument("--yaw", type=float, default=180.0, help="Wrist yaw angle in degrees")
    parser.add_argument("--command-rate", type=float, default=15.0, help="Command streaming rate (Hz)")
    parser.add_argument("--log-level", default="INFO", help="Logging verbosity")
    parser.add_argument("--calibrate", action="store_true", help="Run follower calibration on connect")
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
    return parser.parse_args(list(argv) if argv is not None else None)


def _build_square(
    page_size_m: Sequence[float],
    *,
    square_size_m: float,
    margin_m: float,
) -> list[tuple[float, float]]:
    width_m, height_m = page_size_m
    usable_width = max(width_m - 2 * margin_m, 1e-6)
    usable_height = max(height_m - 2 * margin_m, 1e-6)
    side = min(square_size_m, usable_width, usable_height)
    origin_x = (width_m - side) / 2.0
    origin_y = (height_m - side) / 2.0
    return [
        (origin_x, origin_y),
        (origin_x + side, origin_y),
        (origin_x + side, origin_y + side),
        (origin_x, origin_y + side),
        (origin_x, origin_y),
    ]


def _square_commands(points: Sequence[Sequence[float]]) -> list[DriverCommand]:
    commands: list[DriverCommand] = []
    first = points[0]
    commands.append(PEN_UP())
    commands.append(MOVE_TO(first[0], first[1]))
    commands.append(PEN_DOWN())
    for x, y in points[1:]:
        commands.append(MOVE_TO(x, y))
    commands.append(PEN_UP())
    return commands


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    if args.use_stage_default and args.page_to_robot:
        logging.warning(
            "--use-stage-default supplied; ignoring explicit --page-to-robot path %s",
            args.page_to_robot,
        )
    calibration_path: Path | None
    if args.use_stage_default:
        calibration_path = None
    else:
        calibration_path = args.page_to_robot or DEFAULT_CALIBRATION_PATH
    if calibration_path is not None and not calibration_path.exists():
        raise SystemExit(
            "Calibration file not found at %s. Run `python -m examples.marc.calib.page_to_robot` "
            "to generate it or rerun this command with --use-stage-default to rely on the baked "
            "Cal Hacks transform."
            % calibration_path
        )

    page_width_m = args.page_width_mm / 1000.0
    page_height_m = args.page_height_mm / 1000.0
    page_size_m = (page_width_m, page_height_m)
    square_points = _build_square(
        page_size_m,
        square_size_m=args.square_size_mm / 1000.0,
        margin_m=args.margin_mm / 1000.0,
    )
    commands = _square_commands(square_points)

    cfg = ExecutorConfig(
        travel_speed=args.travel_speed,
        draw_speed=args.draw_speed,
        z_contact=args.z_contact,
        z_safe=args.z_safe,
        command_rate_hz=args.command_rate,
        pitch_deg=args.pitch,
        roll_deg=args.roll,
        yaw_deg=args.yaw,
    )

    driver_kwargs: dict[str, object]
    if args.use_stage_default:
        transform = stage_default_transform((args.page_width_mm, args.page_height_mm))
        driver_kwargs = {"page_to_robot_matrix": transform.matrix}
        logging.info("Using baked stage calibration for page-to-robot transform")
    else:
        driver_kwargs = {"homography_path": calibration_path}

    driver = SO100Driver(
        port=args.port,
        urdf_path=args.urdf,
        page_size=page_size_m,
        executor_cfg=cfg,
        follower_id=args.follower_id,
        calibration_dir=args.follower_calibration_dir,
        **driver_kwargs,
    )

    logging.info(
        "Drawing calibration square with side %.1f mm and %.1f mm margins",
        args.square_size_mm,
        args.margin_mm,
    )

    with driver.session(calibrate=args.calibrate) as session:
        session.execute(commands)

    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())


__all__ = ["main"]
