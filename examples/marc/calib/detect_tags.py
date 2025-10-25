"""Detection helpers for page fiducials or AprilTags."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np


@dataclass
class DetectionResult:
    """Container describing fiducial detection results."""

    page_corners: np.ndarray
    tag_corners: List[np.ndarray]
    tag_ids: np.ndarray
    overlay: np.ndarray
    centers: np.ndarray
    extras: Dict[str, np.ndarray]

    @property
    def homography_ready_corners(self) -> np.ndarray:
        """Return the page corners as ``float32`` for homography computations."""

        return self.page_corners.astype(np.float32)


def _order_points(pts: np.ndarray) -> np.ndarray:
    """Return the four points ordered TL, TR, BR, BL."""

    if pts.shape[0] != 4:
        raise ValueError("Exactly four points are required to order corners")

    pts = pts.astype(np.float32)
    # Sum gives TL (min) and BR (max), diff gives TR (min) and BL (max)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(-1)

    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = pts[np.argmin(s)]
    ordered[2] = pts[np.argmax(s)]
    ordered[1] = pts[np.argmin(diff)]
    ordered[3] = pts[np.argmax(diff)]
    return ordered


def _ensure_color(image: np.ndarray) -> np.ndarray:
    if image.ndim == 3 and image.shape[2] == 3:
        return image.copy()
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    raise ValueError("Expected an image with shape (H, W) or (H, W, 3)")


def detect_fiducials(
    image: np.ndarray,
    *,
    dictionary: str = "DICT_APRILTAG_36h11",
    expected_ids: Optional[Sequence[int]] = None,
    detector_parameters: Optional[cv2.aruco.DetectorParameters] = None,
) -> DetectionResult:
    """Detect four square fiducials in ``image``.

    Parameters
    ----------
    image:
        Input BGR or grayscale image from the calibration camera.
    dictionary:
        Name of the OpenCV ``cv2.aruco`` dictionary to use. The default works for
        AprilTag 36h11 markers which are commonly printed on calibration sheets.
    expected_ids:
        Optional iterable of marker IDs to keep. When provided, only detections
        whose IDs appear in this list are retained. This is useful when the
        calibration target uses known IDs for the page corners.
    detector_parameters:
        Optional ``cv2.aruco.DetectorParameters`` instance to tweak the
        detector. When omitted, OpenCV defaults are used.

    Returns
    -------
    DetectionResult
        Dataclass containing ordered page corners, per-tag information and a
        diagnostic overlay image.

    Raises
    ------
    RuntimeError
        If fewer than four fiducials are detected.
    ValueError
        If ``dictionary`` is not recognised.
    """

    if image.ndim == 3 and image.shape[2] == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    elif image.ndim == 2:
        gray = image
    else:
        raise ValueError("Expected BGR or grayscale image")

    gray = gray.astype(np.uint8)

    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco module is required for fiducial detection")

    dict_attr = getattr(cv2.aruco, dictionary, None)
    if dict_attr is None:
        raise ValueError(f"Unknown cv2.aruco dictionary '{dictionary}'")

    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)
    if detector_parameters is not None:
        params = detector_parameters
    else:
        if hasattr(cv2.aruco, "DetectorParameters"):
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()

    # Detector API differs between OpenCV versions.
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

    if ids is None or len(ids) == 0:
        raise RuntimeError("No fiducials detected in input image")

    ids = ids.reshape(-1)
    if expected_ids is not None:
        expected_set = set(expected_ids)
        filtered: List[np.ndarray] = []
        filtered_ids: List[int] = []
        for marker_corners, marker_id in zip(corners, ids):
            if marker_id in expected_set:
                filtered.append(marker_corners)
                filtered_ids.append(int(marker_id))
        corners = filtered
        ids = np.asarray(filtered_ids, dtype=np.int32)

    tag_corners = [np.asarray(corner).reshape(-1, 2).astype(np.float32) for corner in corners]

    if len(tag_corners) < 4:
        raise RuntimeError("Expected at least four fiducials to determine page corners")

    all_points = np.concatenate(tag_corners, axis=0)

    hull = cv2.convexHull(all_points)
    perimeter = cv2.arcLength(hull, True)
    epsilon = 0.02 * perimeter
    approx = cv2.approxPolyDP(hull, epsilon, True)

    if len(approx) != 4:
        rect = cv2.minAreaRect(all_points)
        approx = cv2.boxPoints(rect)
    else:
        approx = approx.reshape(-1, 2)

    if approx.shape[0] != 4:
        raise RuntimeError("Unable to determine four page corners from detections")

    ordered_corners = _order_points(approx)

    overlay = _ensure_color(image)
    drawn_corners = [corner.reshape(1, -1, 2) for corner in tag_corners]
    cv2.aruco.drawDetectedMarkers(overlay, drawn_corners, ids.reshape(-1, 1))

    centers = np.array([np.mean(corner, axis=0) for corner in tag_corners], dtype=np.float32)
    for center, marker_id in zip(centers, ids):
        cv2.circle(overlay, tuple(int(v) for v in center), 4, (0, 255, 0), -1)
        cv2.putText(
            overlay,
            str(marker_id),
            (int(center[0]) + 4, int(center[1]) - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    overlay_corners = ordered_corners.reshape(4, 1, 2).astype(int)
    cv2.polylines(overlay, [overlay_corners], True, (255, 0, 0), 2, cv2.LINE_AA)

    extras: Dict[str, np.ndarray] = {"hull": hull.reshape(-1, 2)}
    if rejected is not None:
        extras["rejected"] = np.array(rejected, dtype=object)

    return DetectionResult(
        page_corners=ordered_corners,
        tag_corners=tag_corners,
        tag_ids=ids,
        overlay=overlay,
        centers=centers,
        extras=extras,
    )


__all__ = ["DetectionResult", "detect_fiducials"]
