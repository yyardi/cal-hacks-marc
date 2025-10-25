"""Estimate a camera-to-page homography from an overhead image."""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, Sequence

import cv2

from .detect_tags import detect_fiducials
from .homography import DEFAULT_PAGE_SIZE_MM, compute_page_homography, save_homography


def _parse_expected_ids(raw: Iterable[int] | None) -> Sequence[int] | None:
    if raw is None:
        return None
    return list(raw)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Calibration photo with four fiducials")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("examples/marc/out/page_homography.npz"),
        help="Destination file for the homography matrix",
    )
    parser.add_argument(
        "--page-size-mm",
        nargs=2,
        type=float,
        metavar=("WIDTH", "HEIGHT"),
        default=DEFAULT_PAGE_SIZE_MM,
        help="Physical page size in millimetres (default: US Letter)",
    )
    parser.add_argument(
        "--mode",
        choices=("auto", "aruco", "squares"),
        default="squares",
        help="Fiducial detector to use (default: squares)",
    )
    parser.add_argument(
        "--expected-ids",
        type=int,
        nargs="*",
        help="Optional list of AprilTag IDs to keep when detecting fiducials",
    )
    parser.add_argument(
        "--overlay",
        type=Path,
        help="Optional debug image with detections overlaid",
    )
    parser.add_argument(
        "--dictionary",
        default="DICT_APRILTAG_36h11",
        help="OpenCV aruco dictionary name (auto/aruco modes)",
    )
    parser.add_argument(
        "--square-min-area-fraction",
        type=float,
        default=0.0002,
        help="Minimum contour area (as a fraction of the image) for square detection",
    )
    parser.add_argument(
        "--square-max-aspect-ratio",
        type=float,
        default=1.5,
        help="Maximum side-length ratio allowed when accepting square contours",
    )
    parser.add_argument(
        "--square-relaxation-multiplier",
        type=float,
        default=0.25,
        help="(kept for compatibility; not used directly in the new detector)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    image = cv2.imread(str(args.image))
    if image is None:
        raise FileNotFoundError(f"Unable to read calibration image: {args.image}")

    try:
        detection = detect_fiducials(
            image,
            mode=args.mode,
            dictionary=args.dictionary if args.mode != "squares" else None,
            expected_ids=_parse_expected_ids(args.expected_ids),
            square_min_area_fraction=args.square_min_area_fraction,
            square_max_aspect_ratio=args.square_max_aspect_ratio,
            square_relaxation_multiplier=args.square_relaxation_multiplier,
        )
    except Exception as exc:
        raise SystemExit(f"[homography] detection failed: {exc}")

    homography = compute_page_homography(detection.page_corners, tuple(args.page_size_mm))
    save_homography(args.output, homography.homography, homography.page_size)

    if args.overlay:
        args.overlay.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.overlay), detection.overlay)

    print(f"Homography saved to {args.output.resolve()}")
    if args.overlay:
        print(f"Debug overlay saved to {args.overlay.resolve()}")
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())


__all__ = ["main"]
