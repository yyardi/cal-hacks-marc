"""Homography helpers for the calibration workflow."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Tuple

import numpy as np
import cv2

DEFAULT_PAGE_SIZE_IN: Tuple[float, float] = (8.5, 11.0)
DEFAULT_PAGE_SIZE_MM: Tuple[float, float] = tuple(v * 25.4 for v in DEFAULT_PAGE_SIZE_IN)


@dataclass
class HomographyResult:
    """Bundle of camera-to-page homography data."""
    homography: np.ndarray
    page_corners: np.ndarray
    page_size: Tuple[float, float]
    def save(self, path: Path | str) -> None:
        save_homography(path, self.homography, self.page_size)


def page_canonical_corners(page_size: Tuple[float, float] = DEFAULT_PAGE_SIZE_MM) -> np.ndarray:
    """Return canonical page corner coordinates (TL, TR, BR, BL) in mm."""
    width, height = page_size
    return np.array([[0.0, 0.0], [width, 0.0], [width, height], [0.0, height]], dtype=np.float32)


def compute_page_homography(
    image_corners: Iterable[Iterable[float]],
    page_size: Tuple[float, float] = DEFAULT_PAGE_SIZE_MM,
) -> HomographyResult:
    """Compute homography from camera pixels to page coordinates, robustly."""
    image_corners = np.asarray(list(image_corners), dtype=np.float32)
    if image_corners.shape != (4, 2):
        raise ValueError("Expected four 2D points for image_corners")

    page_corners = page_canonical_corners(page_size)
    H, status = cv2.findHomography(image_corners, page_corners, method=cv2.RANSAC, ransacReprojThreshold=3.0)
    if H is None:
        raise RuntimeError("cv2.findHomography failed to compute a homography")

    return HomographyResult(H.astype(np.float64), page_corners.astype(np.float64), page_size)


def invert_homography(H: np.ndarray) -> np.ndarray:
    if H.shape != (3, 3):
        raise ValueError("Expected a 3x3 homography matrix")
    return np.linalg.inv(H)


def save_homography(path: Path | str, homography: np.ndarray, page_size: Tuple[float, float] = DEFAULT_PAGE_SIZE_MM) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, homography=homography, page_size=page_size)


def load_homography(path: Path | str) -> HomographyResult:
    path = Path(path)
    data = np.load(path, allow_pickle=False)
    H = data["homography"].astype(np.float64)
    page_size = tuple(float(v) for v in data["page_size"])
    page_corners = page_canonical_corners(page_size)
    return HomographyResult(H, page_corners, page_size)
