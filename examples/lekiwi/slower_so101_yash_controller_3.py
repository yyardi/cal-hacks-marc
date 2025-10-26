#!/usr/bin/env python

import time
import traceback
import math
from sympy import sin, cos, nsolve
from sympy.abc import x, y

from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun
from lerobot.robots import Robot


# ======================================================
# Inverse Kinematics (stable version)
# ======================================================
def xyzp_inverse_kinematics(robot: Robot, x0, y0, z0, pitch_rad, l1=0.1159, l2=0.1375, l3=0.17):
    # Use fixed initial guesses to ensure solver stability
    init_guess_t2 = 0.3  # radians
    init_guess_t3 = -0.5  # radians

    r = math.sqrt(x0**2 + y0**2)
    try:
        res = nsolve([
            l1 * sin(x) + l2 * cos(x + y) + l3 * cos(pitch_rad) - r,
            l1 * cos(x) - l2 * sin(x + y) + l3 * sin(pitch_rad) - z0
        ], [x, y], [init_guess_t2, init_guess_t3], prec=5)
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


# ======================================================
# Read angles safely
# ======================================================
def read_angles(robot: Robot):
    start_obs = robot.get_observation()
    start_positions = {}
    for key, value in start_obs.items():
        if key.endswith('.pos'):
            motor_name = key.removesuffix('.pos')
            start_positions[motor_name] = value
    return start_positions


# ======================================================
# Smooth movement helper
# ======================================================
def move_smoothly_between_positions(robot: Robot, start_angles, target_angles, steps=60, total_duration=3.0):
    delay = total_duration / float(steps)
    joint_keys = [k for k in start_angles.keys() if k in target_angles]

    for i in range(steps + 1):
        t = i / float(steps)
        interp_action = {}
        for joint in joint_keys:
            interp_val = start_angles[joint] + (target_angles[joint] - start_angles[joint]) * t
            interp_action[f"{joint}.pos"] = interp_val

        interp_action["x.vel"] = 0.0
        interp_action["y.vel"] = 0.0
        interp_action["theta.vel"] = 0.0

        robot.send_action(interp_action)
        time.sleep(delay)


# ======================================================
# Move function (slower and safe)
# ======================================================
def move_to_xyzp(robot: Robot, x, y, z, pitch_deg, move_duration=3.0, smooth=True):
    pitch_rad = math.radians(pitch_deg)

    # Use robust IK
    success, joint_angles = xyzp_inverse_kinematics(robot, x, y, z, pitch_rad)
    if not success:
        print(f"[!] IK failed for target (x={x:.3f}, y={y:.3f}, z={z:.3f}, pitch={pitch_deg}°)")
        return False

    target_position = {
        'shoulder_pan': joint_angles[0],
        'shoulder_lift': joint_angles[1],
        'elbow_flex': joint_angles[2],
        'wrist_flex': joint_angles[3],
        'wrist_roll': 0.0,
        'gripper': 0.0
    }

    print(f"→ Moving to (x={x:.3f}, y={y:.3f}, z={z:.3f}, pitch={pitch_deg:.1f}°)")
    print(f"Joint targets: {joint_angles}")

    if smooth:
        current_angles = read_angles(robot)
        move_smoothly_between_positions(robot, current_angles, target_position, steps=80, total_duration=move_duration)
        # Ensure robot reaches final position before next move
        busy_wait(move_duration)
    else:
        robot.send_action({f"{k}.pos": v for k, v in target_position.items()})
        busy_wait(move_duration)

    print("[✓] Movement complete")
    return True


# ======================================================
# Main
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
    print("Type 'exit' to quit.\n")

    while True:
        try:
            user_input = input("Enter target (x y z pitch_deg): ").strip()
            if user_input.lower() in ["exit", "quit"]:
                break

            parts = user_input.split()
            if len(parts) != 4:
                print("Please enter exactly four values: x y z pitch_deg")
                continue

            x_val, y_val, z_val, pitch_deg = map(float, parts)
            move_to_xyzp(robot, x_val, y_val, z_val, pitch_deg, move_duration=4.0, smooth=True)

        except KeyboardInterrupt:
            print("User interrupted program.")
            break
        except Exception as e:
            print(f"Error: {e}")
            traceback.print_exc()


if __name__ == "__main__":
    main()


