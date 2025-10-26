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

# Stage layout measured on the Cal Hacks demo rig.  All units are metres and
# come from jogging the follower to the page corners shown in the runbook
# photo.  The top-left corner is derived so the transform stays orthogonal to
# the page axes.
DEFAULT_STAGE_ORIGIN_M: tuple[float, float] = (0.195, -0.084)
DEFAULT_STAGE_BOTTOM_RIGHT_M: tuple[float, float] = (0.186, 0.056)
DEFAULT_STAGE_TOP_RIGHT_M: tuple[float, float] = (0.339, 0.060)

# We express the +X (long edge) and +Y (short edge) reference points in page
# coordinates so the calibration helper can feed them straight into the
# Procrustes fit.  +X corresponds to the page coordinate (width, 0) while +Y
# maps to (0, height).
DEFAULT_STAGE_X_POINT_M: tuple[float, float] = (
    DEFAULT_STAGE_TOP_RIGHT_M[0] - (DEFAULT_STAGE_BOTTOM_RIGHT_M[0] - DEFAULT_STAGE_ORIGIN_M[0]),
    DEFAULT_STAGE_TOP_RIGHT_M[1] - (DEFAULT_STAGE_BOTTOM_RIGHT_M[1] - DEFAULT_STAGE_ORIGIN_M[1]),
)
DEFAULT_STAGE_Y_POINT_M: tuple[float, float] = DEFAULT_STAGE_BOTTOM_RIGHT_M

# Convert those to millimetres for convenience.
DEFAULT_STAGE_ORIGIN_MM: tuple[float, float] = tuple(v * 1000.0 for v in DEFAULT_STAGE_ORIGIN_M)
DEFAULT_STAGE_BOTTOM_RIGHT_MM: tuple[float, float] = tuple(
    v * 1000.0 for v in DEFAULT_STAGE_BOTTOM_RIGHT_M
)
DEFAULT_STAGE_TOP_RIGHT_MM: tuple[float, float] = tuple(v * 1000.0 for v in DEFAULT_STAGE_TOP_RIGHT_M)
DEFAULT_STAGE_X_POINT_MM: tuple[float, float] = tuple(v * 1000.0 for v in DEFAULT_STAGE_X_POINT_M)
DEFAULT_STAGE_Y_POINT_MM: tuple[float, float] = tuple(v * 1000.0 for v in DEFAULT_STAGE_Y_POINT_M)

# Z heights and gentle streaming speeds that have proven reliable on stage.
DEFAULT_STAGE_Z_CONTACT: float = -0.028
DEFAULT_STAGE_Z_SAFE: float = 0.05
DEFAULT_STAGE_TRAVEL_SPEED: float = 0.02
DEFAULT_STAGE_DRAW_SPEED: float = 0.01
DEFAULT_STAGE_PICK_SPEED: float = 0.01

# Keep a conservative 12 mm margin on each side when auto-drawing the hardware
# validation square.  This factors in the slight ± drift we see when remounting
# paper between runs.
DEFAULT_STAGE_MARGIN_MM: float = 12.0

__all__ = [
    "SAFE_WORKSPACE_WIDTH_MM",
    "SAFE_WORKSPACE_HEIGHT_MM",
    "SAFE_WORKSPACE_SIZE_MM",
    "SAFE_WORKSPACE_SIZE_M",
    "DEFAULT_STAGE_ORIGIN_M",
    "DEFAULT_STAGE_BOTTOM_RIGHT_M",
    "DEFAULT_STAGE_TOP_RIGHT_M",
    "DEFAULT_STAGE_X_POINT_M",
    "DEFAULT_STAGE_Y_POINT_M",
    "DEFAULT_STAGE_ORIGIN_MM",
    "DEFAULT_STAGE_BOTTOM_RIGHT_MM",
    "DEFAULT_STAGE_TOP_RIGHT_MM",
    "DEFAULT_STAGE_X_POINT_MM",
    "DEFAULT_STAGE_Y_POINT_MM",
    "DEFAULT_STAGE_Z_CONTACT",
    "DEFAULT_STAGE_Z_SAFE",
    "DEFAULT_STAGE_TRAVEL_SPEED",
    "DEFAULT_STAGE_DRAW_SPEED",
    "DEFAULT_STAGE_PICK_SPEED",
    "DEFAULT_STAGE_MARGIN_MM",
]
