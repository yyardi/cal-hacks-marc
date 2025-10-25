"""Fit a rigid transform from page coordinates to robot coordinates."""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from ..constants import SAFE_WORKSPACE_SIZE_MM

OUTPUT_PATH = Path("examples/marc/out/calib_page_to_robot.npy")


@dataclass
class RigidTransform2D:
    rotation: np.ndarray
    translation: np.ndarray

    @property
    def matrix(self) -> np.ndarray:
        mat = np.eye(3, dtype=np.float64)
        mat[:2, :2] = self.rotation
        mat[:2, 2] = self.translation
        return mat

    def save(self, path: Path | str = OUTPUT_PATH) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.save(path, self.matrix)


def load_rigid_transform(path: Path | str) -> RigidTransform2D:
    """Load a :class:`RigidTransform2D` saved with :meth:`RigidTransform2D.save`."""

    matrix = np.load(Path(path))
    if matrix.shape != (3, 3):
        raise ValueError("Expected a 3x3 homogeneous matrix in the .npy file")
    rotation = matrix[:2, :2]
    translation = matrix[:2, 2]
    return RigidTransform2D(rotation, translation)


def fit_rigid_transform(page_points: np.ndarray, robot_points: np.ndarray) -> RigidTransform2D:
    """Find the optimal rigid transform aligning ``page_points`` to ``robot_points``."""

    if page_points.shape != robot_points.shape or page_points.shape[0] < 2:
        raise ValueError("Need matching Nx2 point arrays with N >= 2 to fit a transform")

    src = page_points.astype(np.float64)
    dst = robot_points.astype(np.float64)

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)

    src_centered = src - src_mean
    dst_centered = dst - dst_mean

    H = src_centered.T @ dst_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = dst_mean - src_mean @ R
    return RigidTransform2D(R, t)


def _prompt_point(label: str) -> np.ndarray:
    while True:
        raw = input(f"Enter robot coordinates for {label} (e.g. '123.4 567.8'): ").strip()
        pieces = raw.replace(",", " ").split()
        if len(pieces) != 2:
            print("Please provide exactly two values", file=sys.stderr)
            continue
        try:
            return np.array([float(pieces[0]), float(pieces[1])], dtype=np.float64)
        except ValueError:
            print("Could not parse coordinates, please try again", file=sys.stderr)


def gather_robot_points(labels: Iterable[str]) -> np.ndarray:
    points = [_prompt_point(label) for label in labels]
    return np.vstack(points)


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--page-size-mm",
        nargs=2,
        type=float,
        metavar=("WIDTH", "HEIGHT"),
        default=SAFE_WORKSPACE_SIZE_MM,
        help=(
            "Physical drawing area in millimetres. The default (173×150 mm)"
            " matches the safe SO101 workspace."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=OUTPUT_PATH,
        help=f"Destination .npy file (default: {OUTPUT_PATH})",
    )
    parser.add_argument(
        "--non-interactive",
        action="store_true",
        help="Read robot coordinates from stdin without prompts (three lines, origin/+X/+Y)",
    )
    return parser.parse_args(list(argv) if argv is not None else None)


def read_points_from_stdin() -> np.ndarray:
    values = []
    for label in ("origin", "+X", "+Y"):
        line = sys.stdin.readline()
        if not line:
            raise RuntimeError("Expected three lines of coordinates from stdin")
        pieces = line.replace(",", " ").split()
        if len(pieces) != 2:
            raise RuntimeError(f"Invalid coordinate line for {label!r}: {line.strip()}")
        values.append([float(pieces[0]), float(pieces[1])])
    return np.asarray(values, dtype=np.float64)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)

    page_width, page_height = args.page_size_mm
    page_points = np.array(
        [
            [0.0, 0.0],
            [page_width, 0.0],
            [0.0, page_height],
        ],
        dtype=np.float64,
    )

    print("MARC page-to-robot calibration")
    print("--------------------------------")
    print(
        "Jog the robot to the requested locations and record the XY coordinates"
        " reported by your teleoperation tool."
    )
    print()
    print(
        "Workspace: %.1f mm wide × %.1f mm tall (origin at the lower-left)."
        % (page_width, page_height)
    )
    print(
        "Tip: run `python -m examples.lekiwi.teleoperate` in another terminal"
        " after editing the SO100FollowerConfig port/URDF to match your rig."
    )
    print("Position the pen on each pencil dot, then enter the XY pair here.")

    if args.non_interactive:
        robot_points = read_points_from_stdin()
    else:
        robot_points = gather_robot_points(("origin", "+X", "+Y"))

    transform = fit_rigid_transform(page_points, robot_points)
    transform.save(args.output)

    print("\nCalibration complete.")
    print("Rotation matrix:")
    print(transform.rotation)
    print("Translation vector:")
    print(transform.translation)
    print(f"Saved homogeneous matrix to {args.output.resolve()}")

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "RigidTransform2D",
    "fit_rigid_transform",
    "gather_robot_points",
    "load_rigid_transform",
    "main",
    "OUTPUT_PATH",
]
