#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team.
# Licensed under the Apache License, Version 2.0 (the "License");

import time
import traceback
import math
from sympy import sin, cos, nsolve
from sympy.abc import x, y

from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun
from lerobot.robots import Robot


def xyzp_inverse_kinematics(robot: Robot, x0, y0, z0, pitch_rad, l1=0.1159, l2=0.1375, l3=0.17):
    curr_obs = read_angles(robot)
    curr_t2 = curr_obs["shoulder_pan"] * math.pi / 180
    curr_t3 = curr_obs["elbow_flex"] * math.pi / 180

    r = math.sqrt(x0**2 + y0**2)
    try:
        res = nsolve([
            l1 * sin(x) + l2 * cos(x + y) + l3 * cos(pitch_rad) - r,
            l1 * cos(x) - l2 * sin(x + y) + l3 * sin(pitch_rad) - z0
        ], [x, y], [curr_t2, curr_t3], prec=5)
    except Exception:
        return (False, (0, 0, 0, 0))

    joint1_deg = math.degrees(math.atan2(y0, x0))
    joint2_deg = math.degrees(res[0])
    joint3_deg = math.degrees(res[1])
    joint4_deg = -1 * (joint2_deg + joint3_deg + (pitch_rad * 180 / math.pi))

    THETA_MAX = 100
    THETA_MIN = -100
    joints_degs = (joint1_deg, joint2_deg, joint3_deg, joint4_deg)

    for joint in joints_degs:
        if (joint > THETA_MAX or joint < THETA_MIN):
            return (False, (0, 0, 0, 0))

    return (True, joints_degs)


def move_to_zero_position(robot, duration=3.0):
    zero_position = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0
    }
    move_to_position(robot, zero_position, duration)


def move_to_position(robot: Robot, position, duration=3.0):
    current_positions = read_angles(robot)
    robot_action = {}
    for joint_name, target_pos in position.items():
        if joint_name in current_positions:
            robot_action[f"{joint_name}.pos"] = target_pos

    if robot_action:
        robot_action["x.vel"] = 0.0
        robot_action["y.vel"] = 0.0
        robot_action["theta.vel"] = 0.0
        print("Commanding robot to send action:\n", robot_action)
        robot.send_action(robot_action)

    busy_wait(duration)


def read_angles(robot: Robot):
    start_obs = robot.get_observation()
    start_positions = {}
    for key, value in start_obs.items():
        if key.endswith('.pos'):
            motor_name = key.removesuffix('.pos')
            start_positions[motor_name] = value
    return start_positions


def read_and_print_angles(robot: Robot):
    start_positions = read_angles(robot)
    print("Joint angles:")
    for joint_name, position in start_positions.items():
        print(f"  {joint_name}: {int(position)}°")
    return start_positions


# ======================================================
# New: Move robot using user-input coordinates
# ======================================================
def move_to_xyzp(robot: Robot, x, y, z, pitch_deg, move_duration=3.0):
    """
    Moves the SO101 arm to the desired (x, y, z, pitch) coordinate.
    Args:
        robot: Robot instance (must already be connected)
        x, y, z: Target coordinates in meters (relative to base)
        pitch_deg: Desired pitch in degrees
        move_duration: Time (seconds) to wait after sending command
    """
    pitch_rad = math.radians(pitch_deg)
    success, joint_angles = xyzp_inverse_kinematics(robot, x, y, z, pitch_rad)

    if not success:
        print(f"[!] IK failed for target (x={x:.3f}, y={y:.3f}, z={z:.3f}, pitch={pitch_deg}°)")
        return False

    action = {
        'shoulder_pan.pos': joint_angles[0],
        'shoulder_lift.pos': joint_angles[1],
        'elbow_flex.pos': joint_angles[2],
        'wrist_flex.pos': joint_angles[3],
        'wrist_roll.pos': 0.0,
        'gripper.pos': 0.0
    }

    print(f"→ Moving to (x={x:.3f}, y={y:.3f}, z={z:.3f}, pitch={pitch_deg:.2f}°)")
    print(f"Joint targets: {joint_angles}")

    robot.send_action(action)
    busy_wait(move_duration)

    print("[✓] Movement complete")
    return True


# ======================================================
# Main Function
# ======================================================
def main():
    robot_config = SO100FollowerConfig(port="/dev/ttyACM0", id="marc", use_degrees=True)
    robot = SO100Follower(robot_config)
    robot.connect()

    if not robot.is_connected:
        raise ValueError("Robot failed to connect!")

    init_rerun(session_name="so100_xyz_control")

    print("\nConnected to SO101 successfully.")
    print("Enter coordinates (x, y, z, pitch_deg) to move the robot arm.")
    print("Example: 0.20 0.05 0.10 0  → moves 20cm forward, 5cm left, 10cm high, flat pitch.")
    print("Type 'exit' to quit.\n")

    # Move to zero first
    move_to_zero_position(robot, duration=2)

    while True:
        try:
            user_input = input("Enter target (x y z pitch_deg): ").strip()
            if user_input.lower() in ["exit", "quit"]:
                print("Exiting control loop.")
                break

            parts = user_input.split()
            if len(parts) != 4:
                print("Please enter exactly four values: x y z pitch_deg")
                continue

            x_val, y_val, z_val, pitch_deg = map(float, parts)
            move_to_xyzp(robot, x_val, y_val, z_val, pitch_deg, move_duration=2.5)

        except KeyboardInterrupt:
            print("User interrupted program.")
            break
        except Exception as e:
            print(f"Error: {e}")
            traceback.print_exc()


if __name__ == "__main__":
    main()