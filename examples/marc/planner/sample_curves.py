"""Sampling utilities to convert SVG paths into polylines."""
from __future__ import annotations

from math import ceil
from typing import List, Optional

import numpy as np
from svgpathtools import Path, svg2paths2

from . import Color, SampledPath

try:  # pragma: no cover - matplotlib is optional at runtime
    from matplotlib.colors import to_rgb
except Exception:  # pragma: no cover - fall back to manual parsing
    to_rgb = None


def _parse_rgb_function(value: str) -> Optional[Color]:
    value = value.strip()
    if not value.lower().startswith("rgb") or "(" not in value or ")" not in value:
        return None
    inside = value[value.find("(") + 1 : value.rfind(")")]
    parts = [part.strip() for part in inside.split(",") if part.strip()]
    if len(parts) != 3:
        return None
    channel_values: List[int] = []
    for part in parts:
        if part.endswith("%"):
            try:
                channel = float(part[:-1])
            except ValueError:
                return None
            channel_values.append(int(round(max(0.0, min(100.0, channel)) * 2.55)))
        else:
            try:
                channel = float(part)
            except ValueError:
                return None
            channel_values.append(int(round(max(0.0, min(255.0, channel)))))
    return tuple(channel_values)  # type: ignore[return-value]


def _parse_hex_color(value: str) -> Optional[Color]:
    value = value.strip().lower()
    if value.startswith("#"):
        value = value[1:]
    if len(value) == 3:
        value = "".join(ch * 2 for ch in value)
    if len(value) != 6:
        return None
    try:
        return tuple(int(value[i : i + 2], 16) for i in range(0, 6, 2))  # type: ignore[return-value]
    except ValueError:
        return None


def parse_svg_color(value: Optional[str], default: Color = (0, 0, 0)) -> Color:
    """Parse an SVG color specification into an RGB tuple."""

    if value is None:
        return default
    value = value.strip()
    if not value or value.lower() in {"none", "transparent", "currentcolor"}:
        return default
    if value.lower().startswith("url("):
        return default

    if to_rgb is not None:
        try:
            r, g, b = to_rgb(value)
            return tuple(int(round(channel * 255)) for channel in (r, g, b))  # type: ignore[return-value]
        except ValueError:
            pass

    if value.startswith("#"):
        parsed = _parse_hex_color(value)
        if parsed is not None:
            return parsed

    if value.lower().startswith("rgb"):
        parsed = _parse_rgb_function(value)
        if parsed is not None:
            return parsed

    # Basic SVG color keywords to keep dependency footprint low.
    named_colors = {
        "black": (0, 0, 0),
        "white": (255, 255, 255),
        "red": (255, 0, 0),
        "green": (0, 128, 0),
        "blue": (0, 0, 255),
        "yellow": (255, 255, 0),
        "cyan": (0, 255, 255),
        "magenta": (255, 0, 255),
        "gray": (128, 128, 128),
        "grey": (128, 128, 128),
        "orange": (255, 165, 0),
        "purple": (128, 0, 128),
        "brown": (165, 42, 42),
    }
    lowered = value.lower()
    if lowered in named_colors:
        return named_colors[lowered]

    return default


def _sample_path(path: Path, step: float, min_samples: int = 2) -> np.ndarray:
    """Sample a :class:`svgpathtools.Path` into a polyline."""

    if step <= 0:
        raise ValueError("step must be positive")

    length = path.length(error=1e-4)
    if length == 0:
        point = path.point(0)
        return np.array([[point.real, point.imag]], dtype=float)

    num_segments = max(int(ceil(length / step)), min_samples - 1)
    sample_count = max(min_samples, num_segments + 1)
    ts = np.linspace(0.0, 1.0, sample_count)
    points = np.empty((sample_count, 2), dtype=float)
    for idx, t in enumerate(ts):
        point = path.point(t)
        points[idx, 0] = point.real
        points[idx, 1] = point.imag
    return points


def sample_svg_paths(
    svg_file: str,
    step: float = 2.0,
    min_samples: int = 2,
    color_attribute: str = "stroke",
) -> List[SampledPath]:
    """Load paths from an SVG file and sample them into polylines.

    Parameters
    ----------
    svg_file:
        Path to the SVG document.
    step:
        Desired sampling step size in SVG units.
    min_samples:
        Minimum number of samples per path (defaults to 2).
    color_attribute:
        Attribute name to prioritise when choosing the path color.

    Returns
    -------
    list of :class:`SampledPath`
        The sampled paths with associated color metadata.
    """

    paths, attributes, _ = svg2paths2(svg_file)
    sampled: List[SampledPath] = []

    for path, attr in zip(paths, attributes):
        color_value = attr.get(color_attribute)
        if not color_value or color_value.lower() in {"none", "transparent"}:
            color_value = attr.get("fill")
        color = parse_svg_color(color_value)
        points = _sample_path(path, step=step, min_samples=min_samples)
        if len(points) < 2:
            continue
        sampled.append(
            SampledPath(
                points=points,
                color=color,
                source_color=color,
                attributes={k: str(v) for k, v in attr.items()},
            )
        )

    return sampled


__all__ = ["sample_svg_paths", "parse_svg_color"]
