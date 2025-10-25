"""Utilities for calibrating the MARC drawing robot."""

from .detect_tags import DetectionResult, detect_fiducials
from .homography import (
    DEFAULT_PAGE_SIZE_IN,
    DEFAULT_PAGE_SIZE_MM,
    compute_page_homography,
    load_homography,
    page_canonical_corners,
    save_homography,
)
from .page_to_robot import (
    OUTPUT_PATH as PAGE_TO_ROBOT_OUTPUT_PATH,
    RigidTransform2D,
    fit_rigid_transform,
    load_rigid_transform,
)
from .warp import (
    apply_homography,
    warp_camera_to_page,
    warp_page_to_camera,
    warp_points,
)

__all__ = [
    "DetectionResult",
    "detect_fiducials",
    "DEFAULT_PAGE_SIZE_IN",
    "DEFAULT_PAGE_SIZE_MM",
    "compute_page_homography",
    "load_homography",
    "page_canonical_corners",
    "save_homography",
    "RigidTransform2D",
    "fit_rigid_transform",
    "load_rigid_transform",
    "PAGE_TO_ROBOT_OUTPUT_PATH",
    "apply_homography",
    "warp_camera_to_page",
    "warp_page_to_camera",
    "warp_points",
]
