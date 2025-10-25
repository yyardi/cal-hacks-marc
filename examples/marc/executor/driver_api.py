"""High-level driver interface shared across MARC executors."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Iterable, Sequence


class DriverCommandType(Enum):
    """Enumeration of high-level driver intents."""

    MOVE_TO = auto()
    PEN_DOWN = auto()
    PEN_UP = auto()
    PICK_MARKER = auto()
    RETURN_MARKER = auto()


@dataclass(slots=True)
class ExecutorConfig:
    """Common configuration shared by all driver implementations."""

    travel_speed: float = 80.0
    draw_speed: float = 40.0
    pick_speed: float = 20.0
    z_contact: float = -0.01
    z_safe: float = 0.05
    marker_z_offset: float = 0.02
    command_rate_hz: float = 15.0
    pitch_deg: float = -90.0
    roll_deg: float = 0.0
    yaw_deg: float = 180.0
    marker_slots: dict[str, "MarkerSlot"] = field(default_factory=dict)


@dataclass(slots=True)
class MarkerSlot:
    """Description of a marker slot on the page."""

    color: str
    pick_xy: Sequence[float]
    return_xy: Sequence[float] | None = None
    pickup_contact_z: float | None = None


@dataclass(slots=True)
class Stroke:
    """A single planar stroke described by an ordered list of points."""

    points: Sequence[Sequence[float]]

    def __post_init__(self) -> None:  # pragma: no cover - simple validation
        if len(self.points) == 0:
            raise ValueError("A stroke must contain at least one point")


@dataclass(slots=True)
class DriverCommand:
    """Container for a high-level driver command."""

    type: DriverCommandType
    args: tuple
    kwargs: dict

    def __iter__(self):  # pragma: no cover - convenience for tuple unpacking
        yield self.type
        yield self.args
        yield self.kwargs


def MOVE_TO(x: float, y: float, *, z: float | None = None, speed: float | None = None) -> DriverCommand:
    """Move the end-effector to a target point on the page."""

    return DriverCommand(DriverCommandType.MOVE_TO, (float(x), float(y)), {"z": z, "speed": speed})


def PEN_DOWN() -> DriverCommand:
    """Engage the pen on the page."""

    return DriverCommand(DriverCommandType.PEN_DOWN, tuple(), {})


def PEN_UP() -> DriverCommand:
    """Lift the pen from the page."""

    return DriverCommand(DriverCommandType.PEN_UP, tuple(), {})


def pick_marker(color: str) -> DriverCommand:
    """Request picking a marker of a given color."""

    return DriverCommand(DriverCommandType.PICK_MARKER, (color,), {})


def return_marker(color: str) -> DriverCommand:
    """Return the active marker to its slot."""

    return DriverCommand(DriverCommandType.RETURN_MARKER, (color,), {})


def expand_strokes(strokes: Iterable[Stroke]) -> list[DriverCommand]:
    """Convert strokes to a list of motion commands."""

    commands: list[DriverCommand] = []
    for stroke in strokes:
        commands.append(PEN_UP())
        start = stroke.points[0]
        commands.append(MOVE_TO(start[0], start[1]))
        commands.append(PEN_DOWN())
        for point in stroke.points[1:]:
            commands.append(MOVE_TO(point[0], point[1]))
        commands.append(PEN_UP())
    return commands


__all__ = [
    "DriverCommand",
    "DriverCommandType",
    "ExecutorConfig",
    "MarkerSlot",
    "Stroke",
    "MOVE_TO",
    "PEN_DOWN",
    "PEN_UP",
    "expand_strokes",
    "pick_marker",
    "return_marker",
]
