"""
XLeRobot's Keyboard End-Effector Control page was used as a starting point for this solution
https://xlerobot.readthedocs.io/en/latest/software/index.html#keyboard-end-effector-control

Developed by SIGRobotics UIUC
(this is by no means perfect)

Current issues:
 - The SO101 arm's send_action() command does not produce enough torque to overcome the arm's gravity.
    This results in the arm being tilted slightly farther down than it should be when sticking out. Given that this will, at least initially,
    be used for human keyboard control (and a human can react to this), we are not fixing this issue before release.
 - The x, y, z, and pitch desired positions can advance slightly beyond what the robot is physically capable of reaching.
    This can create situations where the (x, y, z, pitch) combination goes out of bounds and then comes back in bounds somewhere else,
    resulting in the robot jerking to the new position. This is exacerbated by the numerical solver sometimes finding a different (but incorrect) solution
"""

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import _init_rerun

from lerobot.robots import Robot

import time
import traceback
import math

from sympy import cos, sin, nsolve
from sympy.abc import x, y


def xyzp_inverse_kinematics(robot: Robot, x0, y0, z0, pitch_rad, l1=0.1159, l2=0.1375, l3=0.195):
    """
    Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets
    
    Parameters:
        x: End effector x coordinate (forward from the base of the arm is positive)
        y: End effector y coordinate (positive y is to the arm's left (from the arm's perspective))
        z: End effector z coordiante
        pitch_rad: the desired pitch in the world frame in radians.
        l1: Upper arm length (default 0.1159 m)
        l2: Lower arm length (default 0.1375 m)
        l3: Wrist joint to end effector tip/control point length (default 0.195 m)
        
    Returns:
        ((success/fail, (joint1, joint2, joint3, joint4)): Joint angles in degrees as defined in the URDF file
    """

    # Use the current angles for the 2nd and 3rd joints for the numerical solver's starting location. 
    curr_obs = read_angles(robot)
    curr_t2 = curr_obs["arm_shoulder_pan"] * math.pi / 180
    curr_t3 = curr_obs["arm_elbow_flex"] * math.pi / 180


    r = math.sqrt(x0**2 + y0**2)
    try:
        res = nsolve([l1 * sin(x) + l2 * cos(x + y) + l3 * cos(pitch_rad) - r, l1 * cos(x) - l2 * sin(x + y) + l3 * sin(pitch_rad) - z0], [x, y], [curr_t2, curr_t3], prec=5)
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
            # numerical solver returned a bad solution (the robot cannot physically reach it).
            # therefore, we say the IK solver failed.
            return (False, (0, 0, 0, 0)) 

    return (True, joints_degs)


def move_to_zero_position(robot, duration=3.0):
    """
    Move the robot to (what it thinks) is the zero/"middle" position.
    
    Args:
        robot: robot instance
        duration: time to move to zero position (seconds)
    """
    
    zero_position = {'arm_shoulder_pan': 0.0, 'arm_shoulder_lift': 0.0, 'arm_elbow_flex': 0.0, 'arm_wrist_flex': 0.0, 'arm_wrist_roll': 0.0, 'arm_gripper': 0.0}
    move_to_position(robot, zero_position, duration)


def move_to_position(robot: Robot, position, duration=3.0):
    """
    Moves to the specified position. Any unspecified joints will be maintained. Currently waits for 3 seconds for action to finish.
    
    Args:
        robot: robot instance
        start_positions: joint position dictionary
    """
    

    current_positions = read_angles(robot)
    robot_action = {}
    for joint_name, target_pos in position.items():
            if joint_name in current_positions:
                robot_action[f"{joint_name}.pos"] = target_pos
    
    # Send action to robot
    if robot_action:
        robot_action["x.vel"] = 0.0
        robot_action["y.vel"] = 0.0
        robot_action["theta.vel"] = 0.0

        print("commanding robot to send action:\n", robot_action)
        robot.send_action(robot_action)

    busy_wait(duration)


def IK_control_loop(robot: Robot, keyboard: KeyboardTeleop, target_positions, xyzp_start_pos, control_freq=50):
    """    
    Args:
        robot: robot instance
        keyboard: keyboard instance
        target_positions: initial target positions for joints not controlled by inverse kinematics (gripper and wrist roll)
        xyzp_start_pos: a tuple containing:
            current_x: current x coordinate (distance "forward" from the base of the robot),
            current_y: current y coordinate (left/right distance, left from the robot's perspective is positive),
            current_z: current z coordiante (z=0 is the same height as the 2nd motor)
            current_pitch: the current world-frame pitch in radians
            - the origin of the system is the shoulder lift joint.
        control_freq: control frequency (Hz)
    """
    current_xyz = {"x" : xyzp_start_pos[0], "y" : xyzp_start_pos[1], "z" : xyzp_start_pos[2]}
    xyz_ranges = {"x" : {"min" : 0.01, "max" : 0.45}, "y" : {"min" : -0.45, "max" : 0.45}, "z" : {"min" : -0.4, "max" : 0.45}}

    control_period = 1.0 / control_freq
    
    pitch = xyzp_start_pos[3]
    pitch_deg_per_sec = 50 # pitch moves at 20 degrees per second (assuming cycles are instant, which they aren't)
    xyz_L1_m_per_sec = 0.2 # speed per direction

    print(f"Starting P control loop, control frequency: {control_freq}Hz")
    
    while True:
        try:
            t0 = time.perf_counter()

            # Get keyboard input
            keyboard_action = keyboard.get_action()
            
            if keyboard_action:
                # Process keyboard input, update target positions
                changed_xyzp = False
                for key in keyboard_action.keys():
                    if key == 'x':
                        print("Exit command detected, returning to zero position...")
                        move_to_zero_position(robot)
                        return
                    
                    # Joint control mapping
                    joint_controls = {
                        'y': ('arm_wrist_roll', -100 * control_period),      # Joint 5 decrease
                        'h': ('arm_wrist_roll', 100 * control_period),       # Joint 5 increase
                        'g': ('arm_gripper', -100 * control_period),         # Joint 6 decrease
                        't': ('arm_gripper', 100 * control_period),          # Joint 6 increase
                    }
                    
                    # xyz coordinate control
                    xyz_controls = {
                        'w': ('x', xyz_L1_m_per_sec * control_period),   # x increase
                        's': ('x', -xyz_L1_m_per_sec * control_period),  # x decrease
                        'd': ('y', xyz_L1_m_per_sec * control_period),   # y increase
                        'a': ('y', -xyz_L1_m_per_sec * control_period),  # y decrease
                        'q': ('z', xyz_L1_m_per_sec * control_period),   # y increase
                        'e': ('z', -xyz_L1_m_per_sec * control_period),  # y decrease
                    }
                    
                    # Pitch control
                    if key == 'r':
                        changed_xyzp = True
                        pitch = min(pitch + (pitch_deg_per_sec * control_period * math.pi / 180), math.pi)
                        print(f"Increase pitch adjustment: {pitch:.3f}")
                    elif key == 'f':
                        changed_xyzp = True
                        pitch = max(pitch - (pitch_deg_per_sec * control_period * math.pi / 180), -math.pi)
                        print(f"Decrease pitch adjustment: {pitch:.3f}")
                    
                    if key in joint_controls:
                        joint_name, delta = joint_controls[key]
                        if joint_name in target_positions:
                            target_positions[joint_name] += delta
                    
                    elif key in xyz_controls:
                        changed_xyzp = True
                        coord, delta = xyz_controls[key]
                        current_xyz[coord] = max(min(current_xyz[coord] + delta, xyz_ranges[coord]["max"]), xyz_ranges[coord]["min"])

                # only run ik once per cycle, and only if we actually changed something.
                if (changed_xyzp):
                    ik_result = xyzp_inverse_kinematics(robot, current_xyz['x'], current_xyz['y'], current_xyz['z'], pitch)
                    if (ik_result[0]):
                        # found a solution, update joints
                        target_positions['arm_shoulder_pan'] = ik_result[1][0]
                        target_positions['arm_shoulder_lift'] = ik_result[1][1]
                        target_positions['arm_elbow_flex'] = ik_result[1][2]
                        target_positions['arm_wrist_flex'] = ik_result[1][3]

            # Create robot action
            robot_action = {}
            for joint_name, target_pos in target_positions.items():
                target_pos = target_positions[joint_name]
                robot_action[f"{joint_name}.pos"] = target_pos
            
            # Send action to robot
            if robot_action:
                # required for LeKiwi
                robot_action["x.vel"] = 0.0
                robot_action["y.vel"] = 0.0
                robot_action["theta.vel"] = 0.0

                print("x:", round(current_xyz['x'], 3), "y:", round(current_xyz['y'], 3), "z:", round(current_xyz['z'], 3), "Pitch (deg):", pitch * 180 / math.pi)
                robot.send_action(robot_action)
            
            busy_wait(max(1.0 / control_freq - (time.perf_counter() - t0), 0.0))
            
        except KeyboardInterrupt:
            print("User interrupted program")
            break
        except Exception as e:
            print(f"Control loop error: {e}")
            traceback.print_exc()
            break


def read_angles(robot: Robot):
    # Read initial joint angles
    start_obs = robot.get_observation()
    start_positions = {}
    for key, value in start_obs.items():
        if key.endswith('.pos'):
            motor_name = key.removesuffix('.pos')
            start_positions[motor_name] = value

    return start_positions


def read_and_print_angles(robot: Robot):
    """
    Prints the robot's current joint angles, and returns an dictionary with those joint angles.
    """
    start_positions = read_angles(robot)
    
    print("Joint angles:")
    for joint_name, position in start_positions.items():
        print(f"  {joint_name}: {int(position)}°")

    return start_positions


def main():

    # Create the robot and teleoperator configurations
    robot_config = LeKiwiClientConfig(remote_ip="192.168.0.133", id="my_lekiwi2")
    teleop_arm_config = SO100LeaderConfig(port="COM5", id="my_awesome_leader_arm")
    keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

    robot = LeKiwiClient(robot_config)
    leader_arm = SO100Leader(teleop_arm_config)
    keyboard = KeyboardTeleop(keyboard_config)

    # To connect you already should have this script running on LeKiwi: `python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=my_awesome_kiwi`
    robot.connect()
    leader_arm.connect()
    keyboard.connect()

    _init_rerun(session_name="lekiwi_teleop")

    if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
        raise ValueError("Robot, leader arm of keyboard is not connected!")


    # IK control code start 
    try:
        
        # Move to zero position
        # I have to hack the zero position a bit for the robot to overcome its own gravity and actually come to rest at zero (after send action, and of course this is not consistent)
        zero_action = {'arm_shoulder_pan.pos': 0.0, 'arm_shoulder_lift.pos': -2.0, 'arm_elbow_flex.pos': -5.0, 'arm_wrist_flex.pos': 0.0, 'arm_wrist_roll.pos': 0.0, 'arm_gripper.pos': 11.0, 'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
        robot.send_action(zero_action)
        busy_wait(3)


        zero_poses = {
        'arm_shoulder_pan': 0.0,
        'arm_shoulder_lift': 0.0,
        'arm_elbow_flex': 0.0,
        'arm_wrist_flex': 0.0,
        'arm_wrist_roll': 0.0,
        'arm_gripper': 0.0
        }
        
        # Initialize x,y coordinate control
        shoulder_to_elbow_len = 0.1159
        elbow_to_wrist_len = 0.1375
        wrist_to_ee_len = 0.195

        # coordinates for the zero position (which the robot starts at)
        x0, y0, z0 = elbow_to_wrist_len + wrist_to_ee_len, 0.0, shoulder_to_elbow_len
        print(f"Initialize end effector position: x={x0:.4f}, y={y0:.4f}, z={z0:.4f}")
        
        
        print("Keyboard control instructions:")
        print("- W/S: X coordinate (forward/backward)")
        print("- A/D: Y coordinate change (left/right)")
        print("- Q/E: Z coordinate change (up/down)")
        print("- R/F: Pitch adjustment increase/decrease (affects arm_wrist_flex)")
        print("- Y/H: Joint 5 (arm_wrist_roll) increase/decrease")
        print("- T/G: Joint 6 (arm_gripper) increase/decrease")
        print("- X: Exit program (return to start position first)")
        print("="*50)
        print("Note: Robot will continuously move to target positions")
        
        # Start
        IK_control_loop(robot, keyboard, zero_poses, (x0, y0, z0, 0), control_freq=50)

    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        print("Please check:")
        print("1. Whether the robot is properly connected")
        print("2. Whether the USB port is correct")
        print("3. Whether you have sufficient permissions to access USB devices")
        print("4. Whether the robot is properly configured")


if (__name__ == "__main__"):
    main()