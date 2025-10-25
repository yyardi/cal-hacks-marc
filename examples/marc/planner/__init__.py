"""Utilities for sampling, quantizing, and ordering SVG paths for MARC plots."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

Color = Tuple[int, int, int]
PointArray = np.ndarray


def color_to_hex(color: Color) -> str:
    """Convert an RGB tuple into a hex string."""
    return "#" + "".join(f"{channel:02x}" for channel in color)


def hex_to_color(value: str) -> Color:
    """Convert a hex string (e.g. ``#ff00aa``) into an RGB tuple."""
    value = value.strip()
    if value.startswith("#"):
        value = value[1:]
    if len(value) == 3:
        value = "".join(ch * 2 for ch in value)
    if len(value) != 6:
        raise ValueError(f"Unsupported hex color: {value!r}")
    return tuple(int(value[i : i + 2], 16) for i in range(0, 6, 2))  # type: ignore[return-value]


def color_distance(c1: Color, c2: Color) -> float:
    """Compute Euclidean distance between two RGB colors in 0-255 space."""
    return float(np.linalg.norm(np.subtract(c1, c2)))


@dataclass
class SampledPath:
    """A polyline sampled from an SVG path."""

    points: PointArray
    color: Color
    source_color: Optional[Color] = None
    attributes: Dict[str, str] = field(default_factory=dict)

    def reversed(self) -> "SampledPath":
        """Return a copy of this path with the point order reversed."""
        return SampledPath(
            points=self.points[::-1].copy(),
            color=self.color,
            source_color=self.source_color,
            attributes=dict(self.attributes),
        )

    def to_dict(self, include_attributes: bool = False) -> Dict[str, object]:
        """Return a JSON-serializable representation of the polyline."""
        payload = {
            "color": color_to_hex(self.color),
            "points": self.points.tolist(),
        }
        if include_attributes and self.attributes:
            payload["attributes"] = dict(self.attributes)
        if self.source_color is not None:
            payload["source_color"] = color_to_hex(self.source_color)
        return payload


@dataclass
class OrderedStroke:
    """A sampled polyline with ordering metadata for execution."""

    points: PointArray
    color: Color
    pen_up: bool
    attributes: Dict[str, object] = field(default_factory=dict)

    def to_dict(self, include_attributes: bool = True) -> Dict[str, object]:
        payload = {
            "color": color_to_hex(self.color),
            "points": self.points.tolist(),
            "pen_up": self.pen_up,
        }
        if include_attributes and self.attributes:
            payload["attributes"] = dict(self.attributes)
        return payload


__all__ = [
    "Color",
    "PointArray",
    "SampledPath",
    "OrderedStroke",
    "color_to_hex",
    "hex_to_color",
    "color_distance",
]
