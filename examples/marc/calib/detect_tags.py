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
    mode: str = "auto",
    dictionary: Optional[str] = "DICT_APRILTAG_36h11",
    expected_ids: Optional[Sequence[int]] = None,
    detector_parameters: Optional[cv2.aruco.DetectorParameters] = None,
    square_min_area_fraction: float = 0.0015,
    square_max_aspect_ratio: float = 1.5,
) -> DetectionResult:
    """Detect four square fiducials in ``image``.

    Parameters
    ----------
    image:
        Input BGR or grayscale image from the calibration camera.
    mode:
        ``"auto"`` (default) will attempt AprilTag/ArUco detection if ``dictionary``
        is provided, otherwise it falls back to high-contrast square detection.
        Use ``"aruco"`` to force AprilTag detection or ``"squares"`` to force the
        contour-based square detector.
    dictionary:
        Name of the OpenCV ``cv2.aruco`` dictionary to use when ``mode`` is
        ``"auto"`` or ``"aruco"``. Set to ``None`` when only square detection is
        desired.
    expected_ids:
        Optional iterable of marker IDs to keep. When provided, only detections
        whose IDs appear in this list are retained. This is useful when the
        calibration target uses known IDs for the page corners.
    detector_parameters:
        Optional ``cv2.aruco.DetectorParameters`` instance to tweak the
        detector. When omitted, OpenCV defaults are used.
    square_min_area_fraction:
        Minimum contour area (relative to the total image area) for a candidate
        square to be retained when using the contour-based detector.
    square_max_aspect_ratio:
        Maximum edge-length ratio allowed when accepting a contour as a square.

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
    overlay = _ensure_color(image)

    mode = mode.lower()

    use_aruco = False
    tag_corners: List[np.ndarray] = []
    ids: np.ndarray
    extras: Dict[str, np.ndarray] = {}

    if mode in {"auto", "aruco"} and dictionary is not None:
        if not hasattr(cv2, "aruco"):
            if mode == "aruco":
                raise RuntimeError("cv2.aruco module is required for fiducial detection")
        else:
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

            if hasattr(cv2.aruco, "ArucoDetector"):
                detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                corners, raw_ids, rejected = detector.detectMarkers(gray)
            else:
                corners, raw_ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

            if raw_ids is not None and len(raw_ids) > 0:
                use_aruco = True
                raw_ids = raw_ids.reshape(-1)
                if expected_ids is not None:
                    expected_set = set(expected_ids)
                    filtered: List[np.ndarray] = []
                    filtered_ids: List[int] = []
                    for marker_corners, marker_id in zip(corners, raw_ids):
                        if marker_id in expected_set:
                            filtered.append(marker_corners)
                            filtered_ids.append(int(marker_id))
                    corners = filtered
                    raw_ids = np.asarray(filtered_ids, dtype=np.int32)
                tag_corners = [np.asarray(corner).reshape(-1, 2).astype(np.float32) for corner in corners]
                ids = raw_ids.astype(np.int32)
                extras["rejected"] = np.array(rejected, dtype=object) if rejected is not None else np.array([])

    if not use_aruco:
        # Contour-based detector for plain black squares.
        height, width = gray.shape[:2]
        image_area = float(height * width)
        min_area = max(square_min_area_fraction * image_area, 1.0)

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: List[np.ndarray] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            approx = approx.reshape(-1, 2).astype(np.float32)
            edges = np.linalg.norm(np.roll(approx, -1, axis=0) - approx, axis=1)
            min_edge = edges.min()
            max_edge = edges.max()
            if min_edge <= 0:
                continue
            ratio = max_edge / min_edge
            if ratio > square_max_aspect_ratio:
                continue
            candidates.append((area, approx))

        if len(candidates) < 4:
            raise RuntimeError("Detected fewer than four square fiducials; check contrast and framing")

        candidates.sort(key=lambda item: item[0], reverse=True)
        tag_corners = [candidate[1] for candidate in candidates[:4]]
        ids = np.arange(len(tag_corners), dtype=np.int32)
        extras["threshold"] = thresh

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

    centers = np.array([np.mean(corner, axis=0) for corner in tag_corners], dtype=np.float32)

    if use_aruco:
        drawn_corners = [corner.reshape(1, -1, 2) for corner in tag_corners]
        cv2.aruco.drawDetectedMarkers(overlay, drawn_corners, ids.reshape(-1, 1))
    else:
        for idx, corner in enumerate(tag_corners):
            pts = corner.reshape(-1, 1, 2).astype(int)
            cv2.polylines(overlay, [pts], True, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(
                overlay,
                f"sq{idx}",
                (int(centers[idx, 0]) + 4, int(centers[idx, 1]) - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

    overlay_corners = ordered_corners.reshape(4, 1, 2).astype(int)
    cv2.polylines(overlay, [overlay_corners], True, (255, 0, 0), 2, cv2.LINE_AA)

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

    extras["hull"] = hull.reshape(-1, 2)

    return DetectionResult(
        page_corners=ordered_corners,
        tag_corners=tag_corners,
        tag_ids=ids,
        overlay=overlay,
        centers=centers,
        extras=extras,
    )


__all__ = ["DetectionResult", "detect_fiducials"]
