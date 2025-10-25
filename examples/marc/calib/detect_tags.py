# examples/marc/calib/detect_tags.py
"""Detection helpers for page fiducials or AprilTags."""
from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import Dict, List, Optional, Sequence, Iterable, Tuple

import cv2
import numpy as np


@dataclass
class DetectionResult:
    """Container describing fiducial detection results."""

    page_corners: np.ndarray          # (4, 2) float32 ordered TL, TR, BR, BL
    tag_corners: List[np.ndarray]     # list of 4 quads, each (4, 2) float32
    tag_ids: np.ndarray               # (4,) int32
    overlay: np.ndarray               # BGR debug image
    centers: np.ndarray               # (4, 2) float32
    extras: Dict[str, np.ndarray]

    @property
    def homography_ready_corners(self) -> np.ndarray:
        return self.page_corners.astype(np.float32)


# --------------------------- helpers ---------------------------------- #

def _ensure_color(image: np.ndarray) -> np.ndarray:
    if image.ndim == 3 and image.shape[2] == 3:
        return image.copy()
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    raise ValueError("Expected an image with shape (H, W) or (H, W, 3)")


def _order_points(pts: np.ndarray) -> np.ndarray:
    """Order 4 points TL, TR, BR, BL."""
    if pts.shape[0] != 4:
        raise ValueError("Exactly four points are required to order corners")
    pts = pts.astype(np.float32)
    s = pts.sum(axis=1)
    d = (pts[:, 0] - pts[:, 1])
    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = pts[np.argmin(s)]   # TL
    ordered[2] = pts[np.argmax(s)]   # BR
    ordered[1] = pts[np.argmin(d)]   # TR
    ordered[3] = pts[np.argmax(d)]   # BL
    return ordered


def _centroid(quad: np.ndarray) -> tuple[float, float]:
    p = np.asarray(quad, dtype=np.float32).reshape(-1, 2)
    c = p.mean(axis=0)
    return float(c[0]), float(c[1])


def _quad_mean_gray(gray: np.ndarray, quad: np.ndarray) -> float:
    """Average gray value inside a convex quad (lower => darker)."""
    mask = np.zeros_like(gray, dtype=np.uint8)
    pts = quad.reshape(-1, 1, 2).astype(np.int32)
    cv2.fillConvexPoly(mask, pts, 255)
    return float(cv2.mean(gray, mask=mask)[0])


def _pick_four_corners(quads: list[np.ndarray]) -> list[np.ndarray]:
    """
    Choose TL, TR, BL, BR by centroid geometry:
    pick the two smallest y-centroids (top) and two largest y-centroids (bottom),
    then within each pair split by x (left/right).
    """
    if len(quads) < 4:
        raise RuntimeError("Need at least 4 candidate squares")
    centroids = [(*_centroid(q), q) for q in quads]  # (x, y, quad)
    centroids.sort(key=lambda t: t[1])               # by y
    top = sorted(centroids[:2], key=lambda t: t[0])  # by x
    bottom = sorted(centroids[-2:], key=lambda t: t[0])
    TL, TR = top[0][2], top[1][2]
    BL, BR = bottom[0][2], bottom[1][2]
    return [TL, TR, BL, BR]


def _inner_corner_of_square(quad: np.ndarray, page_center: np.ndarray) -> np.ndarray:
    """
    Return the vertex of `quad` that is closest to the page center.
    This is the true 'inner' corner regardless of the square's contour ordering.
    """
    q = np.asarray(quad, dtype=np.float32).reshape(-1, 2)
    d = np.linalg.norm(q - page_center[None, :], axis=1)
    return q[np.argmin(d)]


# ------------------------------ main API ------------------------------------ #

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
    """Detect four fiducials and return ordered page corners."""

    # --- normalize inputs ---
    if image.ndim == 3 and image.shape[2] == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    elif image.ndim == 2:
        gray = image
    else:
        raise ValueError("Expected BGR or grayscale image")
    gray = gray.astype(np.uint8)
    overlay = _ensure_color(image)

    mode = (mode or "auto").lower()

    tag_corners: List[np.ndarray] = []
    ids: np.ndarray = np.empty((0,), dtype=np.int32)
    extras: Dict[str, np.ndarray] = {}
    use_aruco = False

    # --- ArUco / AprilTag path (optional) ---
    if mode in {"auto", "aruco"} and dictionary is not None and hasattr(cv2, "aruco"):
        dict_attr = getattr(cv2.aruco, dictionary, None)
        if dict_attr is None and mode == "aruco":
            raise ValueError(f"Unknown cv2.aruco dictionary '{dictionary}'")
        if dict_attr is not None:
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)
            if detector_parameters is not None:
                params = detector_parameters
            else:
                params = cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, "DetectorParameters") \
                    else cv2.aruco.DetectorParameters_create()
            if hasattr(cv2.aruco, "ArucoDetector"):
                detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                corners, raw_ids, rejected = detector.detectMarkers(gray)
            else:  # legacy API
                corners, raw_ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

            if raw_ids is not None and len(raw_ids) > 0:
                use_aruco = True
                raw_ids = raw_ids.reshape(-1)
                if expected_ids is not None:
                    keep = set(int(v) for v in expected_ids)
                    filt_c, filt_i = [], []
                    for c, i in zip(corners, raw_ids):
                        if int(i) in keep:
                            filt_c.append(c)
                            filt_i.append(int(i))
                    corners = filt_c
                    raw_ids = np.asarray(filt_i, dtype=np.int32)
                tag_corners = [np.asarray(c).reshape(-1, 2).astype(np.float32) for c in corners][:4]
                ids = raw_ids.astype(np.int32)[:4]
                extras["rejected"] = np.array(rejected, dtype=object) if rejected is not None else np.array([])

    if not use_aruco:
        # --- robust black-square contour detector ---
        H, W = gray.shape[:2]
        image_area = float(H * W)
        min_area = max(square_min_area_fraction * image_area, 1.0)

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # build candidate binaries to survive low contrast
        binaries: List[np.ndarray] = []
        _, t_inv = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        binaries.append(t_inv)
        _, t = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binaries.append(t)
        adaptive = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                         cv2.THRESH_BINARY_INV, 31, 5)
        binaries.append(adaptive)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binaries.extend(cv2.morphologyEx(b, cv2.MORPH_CLOSE, kernel) for b in binaries[:2])

        strong: List[tuple[float, np.ndarray]] = []  # (area, quad)
        pool:   List[tuple[float, np.ndarray]] = []
        seen: Dict[tuple[int, ...], tuple[float, np.ndarray]] = {}

        for binary in binaries:
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area <= 0:
                    continue
                perim = cv2.arcLength(contour, True)
                if perim <= 0:
                    continue
                approx = cv2.approxPolyDP(contour, 0.04 * perim, True)
                if len(approx) != 4 or not cv2.isContourConvex(approx):
                    continue
                quad = approx.reshape(-1, 2).astype(np.float32)
                edges = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)
                min_edge = edges.min()
                max_edge = edges.max()
                if min_edge <= 0:
                    continue
                ratio = max_edge / max(min_edge, 1e-6)
                if ratio > square_max_aspect_ratio:
                    continue
                key = tuple(int(v) for v in quad.reshape(-1))
                candidate = (area, quad)
                pool.append(candidate)
                seen[key] = candidate
                if area >= min_area:
                    strong.append(candidate)

        # relax area if needed
        if len(strong) < 4:
            relaxed_min = max(square_relaxation_multiplier * min_area, 1.0)
            strong = [c for c in pool if c[0] >= relaxed_min]

        if len(strong) < 4 and seen:
            logging.getLogger(__name__).warning(
                "Square detector found only %d strong candidates; relaxing thresholds.", len(strong)
            )
            strong = list(seen.values())

        if not strong:
            raise RuntimeError(
                "Detected fewer than four square fiducials; adjust lighting or --square-min-area-fraction."
            )

        # Prefer darker (black) quads and reasonably large ones; take top-K to feed the corner picker
        def _rank(q: np.ndarray) -> Tuple[float, float]:
            return (_quad_mean_gray(gray, q), -cv2.contourArea(q.reshape(-1, 1, 2)))

        ranked_quads = sorted((q for _, q in strong), key=_rank)
        candidates = ranked_quads[:8] if len(ranked_quads) > 8 else ranked_quads

        if len(candidates) < 4:
            raise RuntimeError("Detected fewer than four square fiducials; capture a clearer photo.")

        try:
            tl_sq, tr_sq, bl_sq, br_sq = _pick_four_corners(candidates)
        except Exception:
            largest = sorted(candidates, key=lambda q: cv2.contourArea(q.reshape(-1, 1, 2)), reverse=True)[:4]
            tl_sq, tr_sq, bl_sq, br_sq = _pick_four_corners(largest)

        # --------- CRITICAL FIX: choose the *inner* corner by proximity to global center ---------
        centers_all = np.array([_centroid(q) for q in (tl_sq, tr_sq, bl_sq, br_sq)], dtype=np.float32)
        page_center = centers_all.mean(axis=0)

        tl = _inner_corner_of_square(tl_sq, page_center)
        tr = _inner_corner_of_square(tr_sq, page_center)
        br = _inner_corner_of_square(br_sq, page_center)
        bl = _inner_corner_of_square(bl_sq, page_center)

        tag_corners = [tl_sq, tr_sq, bl_sq, br_sq]
        ids = np.arange(4, dtype=np.int32)
        ordered_corners = np.stack([tl, tr, br, bl], axis=0).astype(np.float32)
        centers = np.array([np.mean(c, axis=0) for c in tag_corners], dtype=np.float32)

        # draw overlay for squares mode
        for idx, corner in enumerate(tag_corners):
            pts = corner.reshape(-1, 1, 2).astype(int)
            cv2.polylines(overlay, [pts], True, (0, 255, 255), 2, cv2.LINE_AA)
            c = centers[idx]
            cv2.putText(overlay, f"sq{idx}", (int(c[0]) + 4, int(c[1]) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    else:
        # We used ArUco. Build ordered_corners from convex hull of tag quads.
        all_pts = np.concatenate(tag_corners, axis=0)
        hull = cv2.convexHull(all_pts)
        perim = cv2.arcLength(hull, True)
        eps = 0.02 * perim
        approx = cv2.approxPolyDP(hull, eps, True)
        if len(approx) != 4:
            rect = cv2.minAreaRect(all_pts)
            approx = cv2.boxPoints(rect)
        else:
            approx = approx.reshape(-1, 2)
        if approx.shape[0] != 4:
            raise RuntimeError("Unable to determine four page corners from detections")
        ordered_corners = _order_points(approx).astype(np.float32)
        centers = np.array([np.mean(c, axis=0) for c in tag_corners], dtype=np.float32)

        drawn = [c.reshape(1, -1, 2) for c in tag_corners]
        cv2.aruco.drawDetectedMarkers(overlay, drawn, ids.reshape(-1, 1))

    # final overlay
    poly = ordered_corners.reshape(4, 1, 2).astype(int)
    cv2.polylines(overlay, [poly], True, (0, 0, 255), 6, cv2.LINE_AA)
    for i, pt in enumerate(ordered_corners):
        cv2.circle(overlay, (int(pt[0]), int(pt[1])), 8, (0, 0, 255), -1)
        cv2.putText(overlay, f"P{i}", (int(pt[0]) + 6, int(pt[1]) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

    for center, marker_id in zip(centers, ids):
        cv2.circle(overlay, tuple(int(v) for v in center), 4, (0, 255, 0), -1)
        cv2.putText(overlay, str(int(marker_id)), (int(center[0]) + 4, int(center[1]) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    extras["centers_all"] = centers.copy()

    return DetectionResult(
        page_corners=ordered_corners,
        tag_corners=tag_corners,
        tag_ids=ids,
        overlay=overlay,
        centers=centers,
        extras=extras,
    )


__all__ = ["DetectionResult", "detect_fiducials"]
