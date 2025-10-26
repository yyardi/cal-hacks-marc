#!/usr/bin/env python

"""
Robot Movement Library for SO100 Drawing Applications

This module provides a clean interface for moving the SO100 robot to specific
x-y-z-pitch coordinates using inverse kinematics. Designed for drawing and 
path-following applications.

Based on the teleoperate.py IK engine developed by SIGRobotics UIUC.
"""

import math
import time
from typing import Tuple, Optional, Dict
from sympy import cos, sin, nsolve
from sympy.abc import x, y

from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.robots import Robot
from lerobot.utils.robot_utils import busy_wait


class RobotMovementController:
    """
    A controller for moving the SO100 robot to specific positions using inverse kinematics.
    
    Attributes:
        robot: The SO100Follower robot instance
        l1: Upper arm length (shoulder to elbow) in meters
        l2: Lower arm length (elbow to wrist) in meters  
        l3: Wrist to end effector tip length in meters
    """
    
    def __init__(self, robot: Robot, l1: float = 0.1159, l2: float = 0.1375, l3: float = 0.17):
        """
        Initialize the robot movement controller.
        
        Args:
            robot: SO100Follower robot instance (must be connected)
            l1: Upper arm length in meters (default 0.1159)
            l2: Lower arm length in meters (default 0.1375)
            l3: Wrist to end effector length in meters (default 0.17)
        """
        self.robot = robot
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        
        # Joint angle limits
        self.THETA_MAX = 100
        self.THETA_MIN = -100
        
        # Current state tracking
        self._current_wrist_roll = 0.0
        self._current_gripper = 0.0
        
    def inverse_kinematics(self, 
                          x_target: float, 
                          y_target: float, 
                          z_target: float, 
                          pitch_rad: float,
                          verbose: bool = False) -> Tuple[bool, Tuple[float, float, float, float]]:
        """
        Calculate inverse kinematics for the robot arm.
        
        Args:
            x_target: Target x coordinate (forward from base, meters)
            y_target: Target y coordinate (left is positive, meters)
            z_target: Target z coordinate (up is positive, meters)
            pitch_rad: Desired pitch angle in radians (world frame)
            verbose: Print debug information if True
            
        Returns:
            Tuple of (success, (joint1, joint2, joint3, joint4)) where:
                - success: True if IK solution found, False otherwise
                - joint angles: In degrees as defined in URDF
        """
        # Calculate radial distance
        r = math.sqrt(x_target**2 + y_target**2)
        
        # Get current angles for numerical solver starting point
        try:
            curr_obs = self._read_angles()
            curr_t2 = curr_obs.get("shoulder_lift", 0.0) * math.pi / 180
            curr_t3 = curr_obs.get("elbow_flex", 0.0) * math.pi / 180
        except Exception as e:
            if verbose:
                print(f"Warning: Could not read current angles: {e}")
                print("Using default starting position for IK solver")
            # Use reasonable defaults if we can't read current position
            curr_t2 = -75.83 * math.pi / 180  # Safe position shoulder
            curr_t3 = 44.43 * math.pi / 180   # Safe position elbow
        
        # Solve for shoulder_lift and elbow_flex using numerical solver
        try:
            res = nsolve(
                [
                    self.l1 * sin(x) + self.l2 * cos(x + y) + self.l3 * cos(pitch_rad) - r,
                    self.l1 * cos(x) - self.l2 * sin(x + y) + self.l3 * sin(pitch_rad) - z_target
                ],
                [x, y],
                [curr_t2, curr_t3],
                prec=5
            )
        except Exception as e:
            if verbose:
                print(f"IK solver failed to converge: {e}")
                print(f"  Target: x={x_target:.3f}, y={y_target:.3f}, z={z_target:.3f}, r={r:.3f}")
                print(f"  Starting from: shoulder_lift={math.degrees(curr_t2):.1f}°, elbow_flex={math.degrees(curr_t3):.1f}°")
            return (False, (0, 0, 0, 0))
        
        # Convert to degrees
        joint1_deg = math.degrees(math.atan2(y_target, x_target))  # shoulder_pan
        joint2_deg = math.degrees(res[0])  # shoulder_lift
        joint3_deg = math.degrees(res[1])  # elbow_flex
        joint4_deg = -1 * (joint2_deg + joint3_deg + (pitch_rad * 180 / math.pi))  # wrist_flex
        
        joints_degs = (joint1_deg, joint2_deg, joint3_deg, joint4_deg)
        
        # Validate joint angles are within limits
        for i, joint in enumerate(joints_degs):
            if joint > self.THETA_MAX or joint < self.THETA_MIN:
                if verbose:
                    joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex']
                    print(f"IK solution has joint {joint_names[i]} out of bounds: {joint:.1f}°")
                return (False, (0, 0, 0, 0))
        
        return (True, joints_degs)
    
    def move_to(self, 
                x: float, 
                y: float, 
                z: float, 
                pitch_rad: float = 0.0,
                duration: float = 0.5,
                wait: bool = True,
                verbose: bool = False) -> bool:
        """
        Move the robot to a specific (x, y, z, pitch) position.
        
        Args:
            x: Target x coordinate in meters (forward from base)
            y: Target y coordinate in meters (left is positive from robot's perspective)
            z: Target z coordinate in meters (up is positive)
            pitch_rad: Desired pitch angle in radians (default 0.0)
            duration: Time to execute movement in seconds (default 0.5)
            wait: Whether to wait for movement to complete (default True)
            verbose: Print debug information if IK fails (default False)
            
        Returns:
            True if movement was successful, False if IK failed
        """
        # Solve inverse kinematics
        success, (j1, j2, j3, j4) = self.inverse_kinematics(x, y, z, pitch_rad, verbose=verbose)
        
        if not success:
            if not verbose:
                print(f"IK failed for position: x={x:.3f}, y={y:.3f}, z={z:.3f}, pitch={math.degrees(pitch_rad):.1f}°")
            return False
        
        # Create action with IK results and current gripper/wrist state
        action = {
            'shoulder_pan.pos': j1,
            'shoulder_lift.pos': j2,
            'elbow_flex.pos': j3,
            'wrist_flex.pos': j4,
            'wrist_roll.pos': self._current_wrist_roll,
            'gripper.pos': self._current_gripper
        }
        
        # Send action to robot
        self.robot.send_action(action)
        
        if wait:
            busy_wait(duration)
        
        return True
    
    def set_wrist_roll(self, angle_deg: float):
        """
        Set the wrist roll angle.
        
        Args:
            angle_deg: Wrist roll angle in degrees
        """
        self._current_wrist_roll = max(min(angle_deg, self.THETA_MAX), self.THETA_MIN)
    
    def set_gripper(self, position: float):
        """
        Set the gripper position.
        
        Args:
            position: Gripper position (-100 to 100)
        """
        self._current_gripper = max(min(position, self.THETA_MAX), self.THETA_MIN)
    
    def _read_angles(self) -> Dict[str, float]:
        """
        Read current joint angles from the robot.
        
        Returns:
            Dictionary of joint names to positions in degrees
        """
        obs = self.robot.get_observation()
        positions = {}
        for key, value in obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                positions[motor_name] = value
        return positions
    
    def get_current_state(self) -> Dict[str, float]:
        """
        Get the current state of all joints.
        
        Returns:
            Dictionary of joint positions
        """
        return self._read_angles()
    
    def move_to_safe_position(self, duration: float = 3.0):
        """
        Move to a safe starting position using direct joint angles.
        
        This is more robust than using IK because it doesn't depend on the
        current robot position.
        
        Args:
            duration: Time to execute movement in seconds
        """
        # These joint angles correspond to x=0.175, y=0.0, z=0.1, pitch=0.0
        # Pre-calculated to avoid IK dependency during initialization
        safe_angles = {
            'shoulder_pan': 0.0,
            'shoulder_lift': -75.83,
            'elbow_flex': 44.43,
            'wrist_flex': 31.39,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }
        
        print(f"Moving to safe position using direct joint angles...")
        
        # Create action with .pos suffixes
        action = {}
        for joint_name, angle in safe_angles.items():
            action[f"{joint_name}.pos"] = angle
        
        # Send action
        self.robot.send_action(action)
        
        # Update internal state
        self._current_wrist_roll = safe_angles['wrist_roll']
        self._current_gripper = safe_angles['gripper']
        
        # Wait for movement to complete
        busy_wait(duration)
        
        print(f"✓ Moved to safe position")
        return True
    
    def move_to_safe_position_ik(self, duration: float = 3.0):
        """
        Move to safe position using IK (only works if robot is already in reasonable position).
        
        For initial positioning, use move_to_safe_position() instead.
        
        Args:
            duration: Time to execute movement in seconds
        """
        init_xyzp = (0.175, 0.0, 0.1, 0.0)
        print(f"Moving to safe position using IK: x={init_xyzp[0]}, y={init_xyzp[1]}, z={init_xyzp[2]}, pitch={init_xyzp[3]}")
        return self.move_to(*init_xyzp, duration=duration)


class DrawingPath:
    """
    Represents a path for the robot to draw, consisting of a series of waypoints.
    """
    
    def __init__(self, z_contact: float = 0.0, z_safe: float = 0.05):
        """
        Initialize a drawing path.
        
        Args:
            z_contact: Z height when pen is in contact with paper (meters)
            z_safe: Z height when pen is lifted for travel (meters)
        """
        self.waypoints = []
        self.z_contact = z_contact
        self.z_safe = z_safe
    
    def add_point(self, x: float, y: float, pen_down: bool = True, pitch_rad: float = 0.0):
        """
        Add a waypoint to the path.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            pen_down: Whether the pen should be down at this point
            pitch_rad: Pitch angle in radians
        """
        z = self.z_contact if pen_down else self.z_safe
        self.waypoints.append((x, y, z, pitch_rad, pen_down))
    
    def execute(self, controller: RobotMovementController, move_duration: float = 0.3):
        """
        Execute this drawing path using the robot controller.
        
        Args:
            controller: RobotMovementController instance
            move_duration: Time for each movement in seconds
            
        Returns:
            True if all movements succeeded, False otherwise
        """
        print(f"Executing drawing path with {len(self.waypoints)} waypoints")
        
        for i, (x, y, z, pitch, pen_down) in enumerate(self.waypoints):
            status = "DOWN" if pen_down else "UP"
            print(f"  Waypoint {i+1}/{len(self.waypoints)}: x={x:.3f}, y={y:.3f}, z={z:.3f}, pen={status}")
            
            success = controller.move_to(x, y, z, pitch, duration=move_duration)
            if not success:
                print(f"Failed to reach waypoint {i+1}")
                return False
        
        print("Path execution complete")
        return True


def create_rectangle_path(x_min: float, y_min: float, 
                         x_max: float, y_max: float,
                         z_contact: float = 0.0,
                         z_safe: float = 0.05) -> DrawingPath:
    """
    Create a rectangular drawing path.
    
    Args:
        x_min, y_min: Bottom-left corner coordinates
        x_max, y_max: Top-right corner coordinates
        z_contact: Z height for drawing
        z_safe: Z height for travel
        
    Returns:
        DrawingPath object for a rectangle
    """
    path = DrawingPath(z_contact=z_contact, z_safe=z_safe)
    
    # Move to start position (pen up)
    path.add_point(x_min, y_min, pen_down=False)
    
    # Draw rectangle (pen down)
    path.add_point(x_min, y_min, pen_down=True)  # Bottom-left
    path.add_point(x_max, y_min, pen_down=True)  # Bottom-right
    path.add_point(x_max, y_max, pen_down=True)  # Top-right
    path.add_point(x_min, y_max, pen_down=True)  # Top-left
    path.add_point(x_min, y_min, pen_down=True)  # Back to start
    
    # Lift pen
    path.add_point(x_min, y_min, pen_down=False)
    
    return path


def create_line_path(x_start: float, y_start: float,
                     x_end: float, y_end: float,
                     z_contact: float = 0.0,
                     z_safe: float = 0.05,
                     num_points: int = 10) -> DrawingPath:
    """
    Create a straight line drawing path with interpolated points.
    
    Args:
        x_start, y_start: Start point coordinates
        x_end, y_end: End point coordinates
        z_contact: Z height for drawing
        z_safe: Z height for travel
        num_points: Number of interpolated points along the line
        
    Returns:
        DrawingPath object for a line
    """
    path = DrawingPath(z_contact=z_contact, z_safe=z_safe)
    
    # Move to start (pen up)
    path.add_point(x_start, y_start, pen_down=False)
    
    # Draw line with interpolated points (pen down)
    for i in range(num_points + 1):
        t = i / num_points
        x = x_start + t * (x_end - x_start)
        y = y_start + t * (y_end - y_start)
        path.add_point(x, y, pen_down=True)
    
    # Lift pen
    path.add_point(x_end, y_end, pen_down=False)
    
    return path


# Example usage
if __name__ == "__main__":
    # Initialize robot
    robot_config = SO100FollowerConfig(
        port="/dev/ttyACM0",
        id="marc",
        use_degrees=True  # Very important!
    )
    robot = SO100Follower(robot_config)
    robot.connect()
    
    if not robot.is_connected:
        raise ValueError("Robot is not connected!")
    
    # Create controller
    controller = RobotMovementController(robot)
    
    # Move to safe starting position
    controller.move_to_safe_position(duration=3.0)
    
    print("\nExample 1: Move to specific points")
    # Move to a specific point
    success = controller.move_to(x=0.20, y=0.05, z=0.15, pitch_rad=0.0, duration=1.0)
    if success:
        print("Successfully moved to target position")
    
    # Move to another point
    controller.move_to(x=0.25, y=-0.05, z=0.12, pitch_rad=math.radians(-15), duration=1.0)
    
    print("\nExample 2: Draw a rectangle")
    # Create and execute a rectangular path
    rect_path = create_rectangle_path(
        x_min=0.15, y_min=-0.05,
        x_max=0.25, y_max=0.05,
        z_contact=0.0,  # Adjust based on your setup
        z_safe=0.05
    )
    rect_path.execute(controller, move_duration=0.5)
    
    print("\nExample 3: Draw a line")
    # Create and execute a line
    line_path = create_line_path(
        x_start=0.15, y_start=-0.05,
        x_end=0.25, y_end=0.05,
        z_contact=0.0,
        z_safe=0.05,
        num_points=20
    )
    line_path.execute(controller, move_duration=0.3)
    
    # Return to safe position
    controller.move_to_safe_position(duration=3.0)
    
    print("\nDemo complete!")
