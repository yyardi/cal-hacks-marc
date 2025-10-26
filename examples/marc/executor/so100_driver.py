"""Driver that adapts :class:`SO100Follower` to the generic executor API."""

from __future__ import annotations

import logging
import math
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Sequence

import numpy as np

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig

from .driver_api import (
    DriverCommand,
    DriverCommandType,
    ExecutorConfig,
    MarkerSlot,
)

logger = logging.getLogger(__name__)


def load_homography(path: str | Path) -> np.ndarray:
    """Load a 3x3 homography matrix from a variety of file formats."""

    file_path = Path(path)
    if not file_path.exists():
        raise FileNotFoundError(f"Homography file not found: {file_path}")

    if file_path.suffix in {".npy", ".npz"}:
        data = np.load(file_path)
        if isinstance(data, np.lib.npyio.NpzFile):
            for key in ("homography", "H", "matrix"):
                if key in data:
                    matrix = data[key]
                    break
            else:
                raise KeyError(
                    f"Unable to find a homography matrix inside {file_path}. "
                    "Expected one of ['homography', 'H', 'matrix']."
                )
        else:
            matrix = data
    elif file_path.suffix in {".json", ".yaml", ".yml"}:
        import json

        with file_path.open("r", encoding="utf-8") as fh:
            payload = json.load(fh)
        matrix = payload.get("homography") or payload.get("H") or payload.get("matrix")
        if matrix is None:
            raise KeyError(
                f"Unable to find a homography matrix inside {file_path}. "
                "Expected top level keys 'homography', 'H', or 'matrix'."
            )
        matrix = np.asarray(matrix, dtype=float)
    else:
        # Fallback to simple whitespace separated text file
        matrix = np.loadtxt(file_path, dtype=float)

    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError(f"Homography must be 3x3. Got shape {matrix.shape} from {file_path}")
    return matrix


@dataclass(slots=True)
class DriverState:
    """Keeps track of the driver internal state."""

    active_color: str | None = None
    pen_down: bool = False
    last_pose: np.ndarray | None = None
    joint_state: np.ndarray | None = None


class SO100Driver:
    """High-level interface for streaming poses to the SO100/SO101 follower arm."""

    def __init__(
        self,
        *,
        port: str,
        urdf_path: str | Path,
        page_size: Sequence[float],
        executor_cfg: ExecutorConfig | None = None,
        base_pose: Sequence[float] = (0.0, 0.0, 0.0),
        target_frame_name: str = "gripper_frame_link",
        camera_homography_path: str | Path | None = None,
        homography_path: str | Path | None = None,
        page_to_robot_matrix: Sequence[Sequence[float]] | np.ndarray | None = None,
    ) -> None:
        self.cfg = executor_cfg or ExecutorConfig()
        self.page_size = np.array(page_size, dtype=float)
        if self.page_size.shape != (2,):
            raise ValueError("page_size must contain exactly two floats: width and height")

        self.port = port
        if (homography_path is None) == (page_to_robot_matrix is None):
            raise ValueError(
                "Provide exactly one of homography_path or page_to_robot_matrix to SO100Driver"
            )
        if homography_path is not None:
            self.page_to_robot_h = load_homography(homography_path)
        else:
            matrix = np.asarray(page_to_robot_matrix, dtype=float)
            if matrix.shape != (3, 3):
                raise ValueError(
                    "page_to_robot_matrix must be a 3x3 homogeneous transform"
                )
            self.page_to_robot_h = matrix
        self.camera_to_page_h = (
            load_homography(camera_homography_path) if camera_homography_path else None
        )
        self.robot_cfg = SO100FollowerConfig(port=port, use_degrees=True)
        self.robot = SO100Follower(self.robot_cfg)
        self.motor_names: tuple[str, ...] = tuple(self.robot.bus.motors.keys())
        self.kinematics = RobotKinematics(
            str(urdf_path),
            target_frame_name=target_frame_name,
            joint_names=list(self.motor_names),
        )
        self.state = DriverState()
        self.dt = 1.0 / float(self.cfg.command_rate_hz)
        self.base_pose = np.array(base_pose, dtype=float)

    # ------------------------------------------------------------------
    # Context managers
    # ------------------------------------------------------------------
    @contextmanager
    def session(self, calibrate: bool = False) -> Iterator["SO100Driver"]:
        """Context manager that connects to the robot and disconnects on exit."""

        self.connect(calibrate=calibrate)
        try:
            yield self
        finally:
            self.disconnect()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------
    def connect(self, calibrate: bool = False) -> None:
        """Connect to the follower arm and cache its current configuration."""

        logger.info("Connecting to SO100 follower on %s", self.port)
        self.robot.connect(calibrate=calibrate)
        obs = self.robot.get_observation()
        joints = np.array([float(obs[f"{name}.pos"]) for name in self.motor_names], dtype=float)
        self.state.joint_state = joints
        self.state.pen_down = False
        self.state.last_pose = self.kinematics.forward_kinematics(joints)
        logger.info("Robot connected. Initial joints: %s", joints)

    def disconnect(self) -> None:
        """Disconnect from the follower."""

        logger.info("Disconnecting from SO100 follower on %s", self.port)
        self.robot.disconnect()

    # ------------------------------------------------------------------
    # High level command processing
    # ------------------------------------------------------------------
    def execute(self, commands: Iterable[DriverCommand]) -> None:
        """Execute a stream of high level driver commands."""

        for command in commands:
            logger.debug("Processing command: %s", command)
            if command.type is DriverCommandType.MOVE_TO:
                self._cmd_move_to(*command.args, **command.kwargs)
            elif command.type is DriverCommandType.PEN_DOWN:
                self._cmd_pen_state(True)
            elif command.type is DriverCommandType.PEN_UP:
                self._cmd_pen_state(False)
            elif command.type is DriverCommandType.PICK_MARKER:
                self._cmd_pick_marker(command.args[0])
            elif command.type is DriverCommandType.RETURN_MARKER:
                self._cmd_return_marker(command.args[0])
            else:  # pragma: no cover - defensive
                raise ValueError(f"Unsupported driver command: {command.type}")

    # ------------------------------------------------------------------
    # Command handlers
    # ------------------------------------------------------------------
    def _cmd_move_to(
        self,
        x: float,
        y: float,
        *,
        z: float | None = None,
        speed: float | None = None,
        validate: bool = True,
    ) -> None:
        if validate:
            self._assert_within_workspace(x, y)
        pose = self._page_to_pose(x, y, z)
        if self.state.last_pose is None:
            self.state.last_pose = pose.copy()
        travel_speed = speed or (self.cfg.draw_speed if self.state.pen_down else self.cfg.travel_speed)
        self._stream_pose(pose, travel_speed)

    def _cmd_pen_state(self, engage: bool) -> None:
        self.state.pen_down = bool(engage)
        logger.info("Pen %s", "down" if engage else "up")

    def _cmd_pick_marker(self, color: str) -> None:
        logger.info("Picking marker %s", color)
        slot = self._get_marker_slot(color)
        contact_z = slot.pickup_contact_z if slot.pickup_contact_z is not None else self.cfg.z_contact
        self._visit_marker(slot.pick_xy, contact_z)
        self.state.active_color = color

    def _cmd_return_marker(self, color: str) -> None:
        logger.info("Returning marker %s", color)
        slot = self._get_marker_slot(color)
        target = slot.return_xy if slot.return_xy is not None else slot.pick_xy
        contact_z = slot.pickup_contact_z if slot.pickup_contact_z is not None else self.cfg.z_contact
        self._visit_marker(target, contact_z)
        if self.state.active_color == color:
            self.state.active_color = None

    # ------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------
    def _get_marker_slot(self, color: str) -> MarkerSlot:
        if color not in self.cfg.marker_slots:
            raise KeyError(f"No marker slot defined for color '{color}'")
        return self.cfg.marker_slots[color]

    def _visit_marker(self, xy: Sequence[float], contact_z: float) -> None:
        self._cmd_pen_state(False)
        self._cmd_move_to(
            xy[0],
            xy[1],
            z=self.cfg.z_safe,
            speed=self.cfg.travel_speed,
            validate=False,
        )
        self._cmd_move_to(
            xy[0],
            xy[1],
            z=contact_z - self.cfg.marker_z_offset,
            speed=self.cfg.pick_speed,
            validate=False,
        )
        time.sleep(0.25)
        self._cmd_move_to(
            xy[0],
            xy[1],
            z=self.cfg.z_safe,
            speed=self.cfg.travel_speed,
            validate=False,
        )

    def _page_to_pose(self, x: float, y: float, z: float | None) -> np.ndarray:
        page_point = np.array([x, y, 1.0], dtype=float)
        robot_xy_h = self.page_to_robot_h @ page_point
        robot_xy = robot_xy_h[:2] / robot_xy_h[2]
        pose = np.eye(4, dtype=float)
        orientation = _rotation_matrix_from_euler(
            np.deg2rad(self.cfg.roll_deg),
            np.deg2rad(self.cfg.pitch_deg),
            np.deg2rad(self.cfg.yaw_deg),
        )
        pose[:3, :3] = orientation
        pose[:2, 3] = robot_xy[:2]
        pose[2, 3] = self.cfg.z_contact if self.state.pen_down else self.cfg.z_safe
        if z is not None:
            pose[2, 3] = float(z)
        pose[:3, 3] += self.base_pose
        return pose

    # ------------------------------------------------------------------
    # Workspace validation
    # ------------------------------------------------------------------
    def _assert_within_workspace(self, x: float, y: float) -> None:
        eps = 1e-6
        if x < -eps or y < -eps or x > self.page_size[0] + eps or y > self.page_size[1] + eps:
            raise ValueError(
                f"Requested point ({x:.4f}, {y:.4f}) lies outside the configured workspace "
                f"(expected 0 ≤ x ≤ {self.page_size[0]:.4f}, 0 ≤ y ≤ {self.page_size[1]:.4f})."
            )

    def _stream_pose(self, target_pose: np.ndarray, speed: float) -> None:
        if self.state.last_pose is None:
            self.state.last_pose = target_pose.copy()
        start_pose = self.state.last_pose
        delta = target_pose[:3, 3] - start_pose[:3, 3]
        distance = float(np.linalg.norm(delta))
        if distance < 1e-6:
            return
        steps = max(1, int(math.ceil(distance / max(speed, 1e-6) / self.dt)))
        logger.debug("Streaming pose over %s steps", steps)
        for step in range(1, steps + 1):
            alpha = step / steps
            pose = np.eye(4, dtype=float)
            pose[:3, :3] = target_pose[:3, :3]
            pose[:3, 3] = start_pose[:3, 3] + alpha * delta
            self._solve_and_send(pose)
            time.sleep(self.dt)
        self.state.last_pose = target_pose

    def _solve_and_send(self, pose: np.ndarray) -> None:
        current = (
            self.state.joint_state
            if self.state.joint_state is not None
            else np.zeros(len(self.motor_names), dtype=float)
        )
        try:
            solution = self.kinematics.inverse_kinematics(
                current,
                pose,
                position_weight=1.0,
                orientation_weight=0.01,
            )
        except Exception as exc:  # pragma: no cover - defensive logging for field debugging
            logger.error("Inverse kinematics failed: %s", exc)
            raise
        self.state.joint_state = solution
        action = {f"{name}.pos": float(val) for name, val in zip(self.motor_names, solution)}
        action["gripper.pos"] = 0.0
        self.robot.send_action(action)

    # ------------------------------------------------------------------
    # Correction utilities
    # ------------------------------------------------------------------
    def capture_page_image(self) -> np.ndarray:
        """Capture the latest page image from the follower's camera."""

        obs = self.robot.get_observation()
        camera_keys = [key for key in obs if isinstance(key, str) and not key.endswith(".pos")]
        if not camera_keys:
            raise RuntimeError("No camera configured on the SO100 follower")
        image = obs[camera_keys[0]]
        return np.asarray(image)

    def perform_correction(
        self,
        *,
        target_raster: np.ndarray,
        homography_resolution: tuple[int, int],
        threshold: float = 0.2,
        min_component: int = 12,
    ) -> None:
        """Perform an image-based correction pass."""

        try:
            import cv2  # type: ignore[import-not-found]
        except ImportError:  # pragma: no cover - optional dependency
            logger.warning("OpenCV not installed, skipping correction pass")
            return

        captured = self.capture_page_image()
        if self.camera_to_page_h is None:
            logger.warning(
                "Camera homography not provided; skipping correction pass. "
                "Re-run with --camera-homography if you need closed-loop drawing."
            )
            return

        warped = cv2.warpPerspective(captured, self.camera_to_page_h, homography_resolution)
        target_gray = _to_gray(target_raster)
        warped_gray = _to_gray(warped)
        target_norm = target_gray.astype(float) / 255.0
        warped_norm = warped_gray.astype(float) / 255.0
        residual_mask = (target_norm - warped_norm) > threshold
        residual_mask = cv2.medianBlur(residual_mask.astype(np.uint8), 3) > 0
        logger.info("Residual pixels: %d", int(residual_mask.sum()))
        if residual_mask.sum() < min_component:
            logger.info("No significant residual to correct")
            return

        strokes = _mask_to_strokes(residual_mask, self.page_size)
        logger.info("Generated %d residual strokes", len(strokes))
        commands: list[DriverCommand] = []
        for stroke in strokes:
            commands.extend(_stroke_to_commands(stroke))
        if commands:
            self.execute(commands)


# ----------------------------------------------------------------------
# Utility functions for correction
# ----------------------------------------------------------------------

def _to_gray(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return image
    if image.shape[2] == 1:
        return image[:, :, 0]
    return np.mean(image[:, :, :3], axis=2)


def _mask_to_strokes(mask: np.ndarray, page_size: Sequence[float]) -> list[list[tuple[float, float]]]:
    from collections import deque

    mask = mask.astype(bool)
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    scale_x = float(page_size[0]) / max(width - 1, 1)
    scale_y = float(page_size[1]) / max(height - 1, 1)
    strokes: list[list[tuple[float, float]]] = []
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for y in range(height):
        for x in range(width):
            if not mask[y, x] or visited[y, x]:
                continue
            queue: deque[tuple[int, int]] = deque([(y, x)])
            component: list[tuple[int, int]] = []
            while queue:
                cy, cx = queue.popleft()
                if visited[cy, cx] or not mask[cy, cx]:
                    continue
                visited[cy, cx] = True
                component.append((cy, cx))
                for dy, dx in neighbors:
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < height and 0 <= nx < width and not visited[ny, nx]:
                        queue.append((ny, nx))
            component.sort()
            stroke = [(cx * scale_x, cy * scale_y) for cy, cx in component]
            if len(stroke) >= 2:
                strokes.append(stroke)
    return strokes


def _stroke_to_commands(points: list[tuple[float, float]]) -> list[DriverCommand]:
    commands: list[DriverCommand] = [DriverCommand(DriverCommandType.PEN_UP, tuple(), {})]
    first = points[0]
    commands.append(DriverCommand(DriverCommandType.MOVE_TO, (first[0], first[1]), {}))
    commands.append(DriverCommand(DriverCommandType.PEN_DOWN, tuple(), {}))
    for x, y in points[1:]:
        commands.append(DriverCommand(DriverCommandType.MOVE_TO, (x, y), {}))
    commands.append(DriverCommand(DriverCommandType.PEN_UP, tuple(), {}))
    return commands


def _rotation_matrix_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Create a rotation matrix from ZYX Euler angles (roll, pitch, yaw)."""

    sx, cx = np.sin(roll), np.cos(roll)
    sy, cy = np.sin(pitch), np.cos(pitch)
    sz, cz = np.sin(yaw), np.cos(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=float)
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=float)
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=float)

    return rz @ ry @ rx


__all__ = ["SO100Driver", "load_homography"]
