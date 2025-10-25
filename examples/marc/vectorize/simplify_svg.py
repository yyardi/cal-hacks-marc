"""SVG simplification helpers using svgpathtools."""
from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, List, Optional

from svgpathtools import parse_path

LOGGER = logging.getLogger(__name__)

SVG_NAMESPACE = "http://www.w3.org/2000/svg"
ET.register_namespace("", SVG_NAMESPACE)


def _rdp(points: List[complex], epsilon: float) -> List[complex]:
    if len(points) < 3:
        return points

    start = points[0]
    end = points[-1]

    def perpendicular_distance(point: complex) -> float:
        if start == end:
            return abs(point - start)
        num = abs((end.real - start.real) * (start.imag - point.imag) - (start.real - point.real) * (end.imag - start.imag))
        den = math.hypot(end.real - start.real, end.imag - start.imag)
        return num / den

    distances = [perpendicular_distance(p) for p in points[1:-1]]
    if not distances:
        return points
    max_distance = max(distances)
    if max_distance > epsilon:
        index = distances.index(max_distance) + 1
        left = _rdp(points[: index + 1], epsilon)
        right = _rdp(points[index:], epsilon)
        return left[:-1] + right
    return [start, end]


def _sample_path(path, tolerance: float) -> List[List[complex]]:
    sampled_subpaths: List[List[complex]] = []
    for sub_path in path.continuous_subpaths():
        samples: List[complex] = []
        for segment in sub_path:
            length = segment.length(error=1e-4)
            steps = max(int(length / max(tolerance, 1e-6)) + 1, 2)
            for i in range(steps):
                t = i / (steps - 1)
                point = segment.point(t)
                if samples and abs(samples[-1] - point) < 1e-9:
                    continue
                samples.append(point)
        sampled_subpaths.append(samples)
    return sampled_subpaths


def _points_to_path(points: Iterable[complex], closed: bool) -> str:
    iterator = iter(points)
    try:
        first = next(iterator)
    except StopIteration:
        return ""
    commands = [f"M {first.real:.4f} {first.imag:.4f}"]
    for point in iterator:
        commands.append(f"L {point.real:.4f} {point.imag:.4f}")
    if closed:
        commands.append("Z")
    return " ".join(commands)


def simplify_path_d(d: str, tolerance: float = 2.0) -> str:
    try:
        path = parse_path(d)
    except Exception as exc:  # pragma: no cover - only triggered on malformed paths
        LOGGER.warning("Could not parse path: %s", exc)
        return d

    simplified_paths: List[str] = []
    for sub_path in _sample_path(path, tolerance):
        if len(sub_path) < 2:
            continue
        closed = abs(sub_path[0] - sub_path[-1]) < 1e-6
        simplified = _rdp(sub_path, tolerance)
        if closed and simplified[0] != simplified[-1]:
            simplified = simplified + [simplified[0]]
        simplified_paths.append(_points_to_path(simplified, closed))
    return " ".join(simplified_paths)


def simplify_svg_content(svg_text: str, tolerance: float = 2.0) -> str:
    root = ET.fromstring(svg_text)
    for path_elem in root.iter(f"{{{SVG_NAMESPACE}}}path"):
        d = path_elem.get("d")
        if not d:
            continue
        simplified = simplify_path_d(d, tolerance)
        if simplified:
            path_elem.set("d", simplified)
    return ET.tostring(root, encoding="unicode")


def simplify_svg_file(
    input_path: Path,
    output_path: Optional[Path] = None,
    *,
    tolerance: float = 2.0,
) -> Path:
    input_path = Path(input_path)
    svg_text = input_path.read_text(encoding="utf-8")
    simplified = simplify_svg_content(svg_text, tolerance)

    if output_path is None:
        output_path = input_path
    else:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

    output_path.write_text(simplified, encoding="utf-8")
    LOGGER.info("Simplified SVG saved to %s", output_path)
    return output_path
