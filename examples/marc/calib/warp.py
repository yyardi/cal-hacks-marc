"""Warp helpers between camera, page and robot frames."""
from __future__ import annotations

from typing import Iterable, Tuple

import cv2
import numpy as np

from .page_to_robot import RigidTransform2D


def _as_homogeneous(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim == 1:
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        raise ValueError("Expected an array of shape (N, 2)")
    if points.shape[1] != 2:
        raise ValueError("Expected an array of shape (N, 2)")
    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    return np.hstack([points, ones])


def apply_homography(points: Iterable[Iterable[float]], H: np.ndarray) -> np.ndarray:
    """Apply a 3x3 homography ``H`` to an array of ``points``."""

    pts = np.asarray(list(points), dtype=np.float64)
    if pts.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    hom = _as_homogeneous(pts)
    mapped = hom @ H.T
    mapped /= mapped[:, 2:3]
    return mapped[:, :2]


def warp_points(points: Iterable[Iterable[float]], H: np.ndarray) -> np.ndarray:
    """Alias for :func:`apply_homography` for readability."""

    return apply_homography(points, H)


def warp_camera_to_page(
    image: np.ndarray,
    H_camera_to_page: np.ndarray,
    page_size_px: Tuple[int, int],
    *,
    interpolation: int = cv2.INTER_LINEAR,
) -> np.ndarray:
    """Warp a camera image onto the canonical page coordinate frame."""

    width, height = page_size_px
    return cv2.warpPerspective(image, H_camera_to_page, (width, height), flags=interpolation)


def warp_page_to_camera(
    image: np.ndarray,
    H_page_to_camera: np.ndarray,
    image_size: Tuple[int, int],
    *,
    interpolation: int = cv2.INTER_LINEAR,
) -> np.ndarray:
    """Warp a canonical page image back into the camera view."""

    width, height = image_size
    return cv2.warpPerspective(image, H_page_to_camera, (width, height), flags=interpolation)


def apply_rigid_transform(points: Iterable[Iterable[float]], transform: RigidTransform2D | np.ndarray) -> np.ndarray:
    """Apply a rigid transform to ``points``.

    Parameters
    ----------
    points:
        Iterable of ``(x, y)`` coordinates in page space.
    transform:
        Either a :class:`RigidTransform2D` instance or a 3x3 homogeneous matrix
        whose upper-left 2x2 block is a rotation and whose last column is a
        translation.
    """

    pts = np.asarray(list(points), dtype=np.float64)
    if pts.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    if isinstance(transform, RigidTransform2D):
        matrix = transform.matrix
    else:
        matrix = np.asarray(transform, dtype=np.float64)
    hom = _as_homogeneous(pts)
    mapped = hom @ matrix.T
    return mapped[:, :2]


def apply_rigid_inverse(points: Iterable[Iterable[float]], transform: RigidTransform2D | np.ndarray) -> np.ndarray:
    """Apply the inverse rigid transform to ``points``."""

    if isinstance(transform, RigidTransform2D):
        matrix = transform.matrix
    else:
        matrix = np.asarray(transform, dtype=np.float64)
    inv = np.linalg.inv(matrix)
    return apply_homography(points, inv)


__all__ = [
    "apply_homography",
    "apply_rigid_inverse",
    "apply_rigid_transform",
    "warp_camera_to_page",
    "warp_page_to_camera",
    "warp_points",
]
