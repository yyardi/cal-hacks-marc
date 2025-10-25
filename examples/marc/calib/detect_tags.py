"""Detection helpers for page fiducials or AprilTags."""
from __future__ import annotations

from dataclasses import dataclass
import logging
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


def _order_square_quads_by_position(squares: Sequence[np.ndarray]) -> List[np.ndarray]:
    """Return squares ordered as top-left, top-right, bottom-left, bottom-right."""

    if len(squares) < 4:
        raise ValueError("At least four squares are required to determine page corners")

    centers = []
    for square in squares:
        arr = np.asarray(square, dtype=np.float32).reshape(-1, 2)
        if arr.shape[0] != 4:
            continue
        center = arr.mean(axis=0)
        centers.append((float(center[0]), float(center[1]), arr))

    if len(centers) < 4:
        raise ValueError("Unable to compute square centroids; check detections")

    centers.sort(key=lambda item: item[1])  # sort by y (top to bottom)
    top = sorted(centers[:2], key=lambda item: item[0])
    bottom = sorted(centers[2:4], key=lambda item: item[0])

    return [top[0][2], top[1][2], bottom[0][2], bottom[1][2]]


def _page_corners_from_square_fiducials(squares: Sequence[np.ndarray]) -> np.ndarray:
    """Compute page corners from the inner corners of four black squares."""

    candidates: List[tuple[float, np.ndarray]] = []
    for square in squares:
        arr = np.asarray(square, dtype=np.float32).reshape(-1, 2)
        if arr.shape[0] != 4:
            continue
        area = abs(cv2.contourArea(arr.reshape(-1, 1, 2)))
        candidates.append((float(area), arr))

    if len(candidates) < 4:
        raise ValueError("Expected at least four square fiducials for page corner extraction")

    candidates.sort(key=lambda item: item[0], reverse=True)
    top_four = [item[1] for item in candidates[:4]]

    squares_by_position = _order_square_quads_by_position(top_four)
    if len(squares_by_position) != 4:
        raise ValueError("Unable to order square fiducials by position")

    tl_sq, tr_sq, bl_sq, br_sq = squares_by_position

    page_corners = np.stack(
        [
            _order_points(tl_sq)[2],  # inner corner of top-left square
            _order_points(tr_sq)[3],  # inner corner of top-right square
            _order_points(br_sq)[0],  # inner corner of bottom-right square
            _order_points(bl_sq)[1],  # inner corner of bottom-left square
        ],
        axis=0,
    )

    return page_corners.astype(np.float32)


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
    square_min_area_fraction: float = 0.0002,
    square_max_aspect_ratio: float = 1.5,
    square_relaxation_multiplier: float = 0.25,
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
    square_relaxation_multiplier:
        Factor applied to ``square_min_area_fraction`` when the first pass finds
        fewer than four candidates. Smaller factors relax the requirement more
        aggressively before falling back to a best-effort selection of quads.

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

        def _gather_quads(binary: np.ndarray) -> tuple[list[tuple[float, np.ndarray]], list[tuple[float, np.ndarray]]]:
            quads: list[tuple[float, np.ndarray]] = []
            filtered: list[tuple[float, np.ndarray]] = []
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area <= 0:
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
                ratio = max_edge / max(min_edge, 1e-6)
                if ratio > square_max_aspect_ratio:
                    continue
                candidate = (float(area), approx)
                quads.append(candidate)
                if area >= min_area:
                    filtered.append(candidate)
            return filtered, quads

        # Build a pool of candidate binaries so low contrast photos still yield quads.
        binaries = []
        _, otsu_inv = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        binaries.append(otsu_inv)
        _, otsu = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binaries.append(otsu)
        adaptive = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            5,
        )
        binaries.append(adaptive)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binaries.extend(cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel) for binary in binaries[:2])

        filtered_quads: list[tuple[float, np.ndarray]] = []
        all_quads: list[tuple[float, np.ndarray]] = []
        seen: dict[tuple[int, int, int, int], tuple[float, np.ndarray]] = {}

        for binary in binaries:
            filtered, quads = _gather_quads(binary)
            all_quads.extend(quads)
            filtered_quads.extend(filtered)
            for _, approx in quads:
                key = tuple(int(v) for v in approx.reshape(-1))
                seen[key] = (float(cv2.contourArea(approx.reshape(-1, 1, 2))), approx)

        if len(filtered_quads) < 4:
            # Relax area threshold automatically: look for the best quads even if tiny.
            relaxed_min_area = max(square_relaxation_multiplier * min_area, 1.0)
            filtered_quads = [candidate for candidate in all_quads if candidate[0] >= relaxed_min_area]

        if len(filtered_quads) < 4 and seen:
            # Still not enough? take the four largest distinct quads overall to give users a best-effort result.
            logger = logging.getLogger(__name__)
            logger.warning(
                "Square detector only found %d strong candidates; relaxing thresholds. "
                "Consider increasing square size or adjusting --square-min-area-fraction.",
                len(filtered_quads),
            )
            deduped = sorted(seen.values(), key=lambda item: item[0], reverse=True)
            filtered_quads = deduped[:4]

        if len(filtered_quads) < 4:
            # Final fallback: estimate page corners from the largest contour.
            logger = logging.getLogger(__name__)
            logger.warning("Falling back to page contour detection; verify overlay before proceeding.")
            binary = binaries[0]
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                raise RuntimeError(
                    "Detected fewer than four square fiducials; adjust lighting or tweak --square-min-area-fraction"
                )
            largest = max(contours, key=cv2.contourArea)
            epsilon = 0.02 * cv2.arcLength(largest, True)
            approx = cv2.approxPolyDP(largest, epsilon, True).reshape(-1, 2).astype(np.float32)
            if approx.shape[0] < 4:
                rect = cv2.boxPoints(cv2.minAreaRect(largest))
                approx = rect.astype(np.float32)
            if approx.shape[0] != 4:
                raise RuntimeError(
                    "Unable to infer page bounds; capture a clearer photo or increase fiducial size"
                )
            approx = _order_points(approx)
            side = max(np.sqrt(cv2.contourArea(largest)) * 0.05, 5.0)
            offsets = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], dtype=np.float32) * side
            synthetic: list[tuple[float, np.ndarray]] = []
            for point in approx:
                square = point + offsets
                synthetic.append((float(side ** 2), square))
            filtered_quads = synthetic

        filtered_quads.sort(key=lambda item: item[0], reverse=True)
        tag_corners = [candidate[1] for candidate in filtered_quads[:4]]
        ids = np.arange(len(tag_corners), dtype=np.int32)
        extras["threshold"] = otsu_inv

    if len(tag_corners) < 4:
        raise RuntimeError("Expected at least four fiducials to determine page corners")

    ordered_corners: Optional[np.ndarray] = None

    if not use_aruco:
        try:
            ordered_corners = _page_corners_from_square_fiducials(tag_corners)
        except ValueError:
            ordered_corners = None

    if ordered_corners is None:
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
    cv2.polylines(overlay, [overlay_corners], True, (0, 0, 255), 6, cv2.LINE_AA)
    for idx, pt in enumerate(ordered_corners):
        cv2.circle(overlay, (int(pt[0]), int(pt[1])), 8, (0, 0, 255), -1)
        cv2.putText(
            overlay,
            f"P{idx}",
            (int(pt[0]) + 6, int(pt[1]) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

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
