"""Shared constants for the MARC examples."""

from __future__ import annotations

# The SO100/SO101 follower comfortably reaches a rectangle that is roughly
# 173 mm wide by 150 mm tall when the page origin is anchored to the lower-left
# corner of the drawing.  We keep these dimensions in a single module so the
# planner, calibration helpers, and runtime checks all agree on the workspace
# size.
SAFE_WORKSPACE_WIDTH_MM: float = 173.0
SAFE_WORKSPACE_HEIGHT_MM: float = 150.0

# Convenient tuples in both millimetres and metres for CLI defaults.
SAFE_WORKSPACE_SIZE_MM: tuple[float, float] = (
    SAFE_WORKSPACE_WIDTH_MM,
    SAFE_WORKSPACE_HEIGHT_MM,
)
SAFE_WORKSPACE_SIZE_M: tuple[float, float] = (
    SAFE_WORKSPACE_WIDTH_MM / 1000.0,
    SAFE_WORKSPACE_HEIGHT_MM / 1000.0,
)

__all__ = [
    "SAFE_WORKSPACE_WIDTH_MM",
    "SAFE_WORKSPACE_HEIGHT_MM",
    "SAFE_WORKSPACE_SIZE_MM",
    "SAFE_WORKSPACE_SIZE_M",
]
