import time

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig, LeKiwiConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import _init_rerun, log_rerun_data

from lerobot.robots import Robot

import time
import logging
import traceback
import math

from sympy import cos, sin, nsolve
from sympy.abc import x, y


# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Joint calibration coefficients - manually edited
# Format: [joint_name, zero_position_offset(degrees), scale_factor]
# JOINT_CALIBRATION = [
#     ['arm_shoulder_pan', 6.0, 1.0],      # Joint 1: zero position offset, scale factor
#     ['arm_shoulder_lift', 2.0, 0.97],     # Joint 2: zero position offset, scale factor
#     ['arm_elbow_flex', 0.0, 1.05],        # Joint 3: zero position offset, scale factor
#     ['arm_wrist_flex', 0.0, 0.94],        # Joint 4: zero position offset, scale factor
#     ['arm_wrist_roll', 0.0, 0.5],        # Joint 5: zero position offset, scale factor
#     ['arm_gripper', 0.0, 1.0],           # Joint 6: zero position offset, scale factor
# ]

JOINT_CALIBRATION = [
    ['arm_shoulder_pan', 0.0, 1.0],      # Joint 1: zero position offset, scale factor
    ['arm_shoulder_lift', 0.0, 1.0],     # Joint 2: zero position offset, scale factor
    ['arm_elbow_flex', 0.0, 1.0],        # Joint 3: zero position offset, scale factor
    ['arm_wrist_flex', 0.0, 1.0],        # Joint 4: zero position offset, scale factor
    ['arm_wrist_roll', 0.0, 1.0],        # Joint 5: zero position offset, scale factor
    ['arm_gripper', 0.0, 1.0],           # Joint 6: zero position offset, scale factor
]

def apply_joint_calibration(joint_name, raw_position):
    """
    Apply joint calibration coefficients
    
    Args:
        joint_name: joint name
        raw_position: raw position value
    
    Returns:
        calibrated_position: calibrated position value
    """
    for joint_cal in JOINT_CALIBRATION:
        if joint_cal[0] == joint_name:
            offset = joint_cal[1]  # zero position offset
            scale = joint_cal[2]   # scale factor
            calibrated_position = (raw_position - offset) * scale
            return calibrated_position
    return raw_position  # if no calibration coefficient found, return original value


def ik_check_in_bounds(robot: Robot, x, y, z, l1=0.1159, l2=0.1375, l3=0.175):
    # Convert from the end effector's frame to the wrist's frame
    x, y, z = ee_to_wrist_frame(robot, x, y, z, l3)

    r = math.sqrt(x**2 + y**2)

    # Calculate distance from origin to target point
    phi_wrist = math.sqrt(r**2 + z**2)
    phi_wrist_max = l1 + l2  # Maximum reachable distance
    phi_wrist_min = abs(l1 - l2)

    # If target point is beyond maximum workspace, scale it to the boundary
    return not (phi_wrist > phi_wrist_max or phi_wrist < phi_wrist_min)


def inverse_kinematics(r, z, l1=0.1159, l2=0.1350):
    """
    Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets
    
    Parameters:
        x: End effector x coordinate
        y: End effector y coordinate
        l1: Upper arm length (default 0.1159 m)
        l2: Lower arm length (default 0.1350 m)
        
    Returns:
        joint2, joint3: Joint angles in degrees as defined in the URDF file
    """
    # Calculate joint2 and joint3 offsets in theta1 and theta2
    theta1_offset = math.atan2(0.028, 0.11257)  # theta1 offset when joint2=0
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset  # theta2 offset when joint3=0
    
    # Calculate distance from origin to target point
    phi_wrist = math.sqrt(r**2 + z**2)
    phi_wrist_max = l1 + l2  # Maximum reachable distance
    
    # If target point is beyond maximum workspace, scale it to the boundary
    if phi_wrist > phi_wrist_max:
        print("greater than phi_wrist_max")
        scale_factor = phi_wrist_max / phi_wrist
        r *= scale_factor
        z *= scale_factor
        phi_wrist = phi_wrist_max
    
    # If target point is less than minimum workspace (|l1-l2|), scale it
    phi_wrist_min = abs(l1 - l2)
    if phi_wrist < phi_wrist_min and r > 0:
        print("less than r_min")
        scale_factor = phi_wrist_min / phi_wrist
        r *= scale_factor
        z *= scale_factor
        phi_wrist = phi_wrist_min
    
    # Use law of cosines to calculate theta2
    cos_theta2 = -(phi_wrist**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    # Calculate theta2 (elbow angle)
    theta2 = math.pi - math.acos(cos_theta2)
    
    # Calculate theta1 (shoulder angle)
    beta = math.atan2(z, r)
    gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    theta1 = beta + gamma
    
    # Convert theta1 and theta2 to joint2 and joint3 angles
    joint2 = theta1 + theta1_offset
    joint3 = theta2 + theta2_offset
    
    # Ensure angles are within URDF limits
    joint2 = max(-0.1, min(3.45, joint2))
    joint3 = max(-0.2, min(math.pi, joint3))
    
    # Convert from radians to degrees
    joint2_deg = math.degrees(joint2)
    joint3_deg = math.degrees(joint3)

    joint2_deg = 90-joint2_deg
    joint3_deg = joint3_deg-90
    
    return joint2_deg, joint3_deg



def inverse_kinematics_TWO(r, z, l1=0.1159, l2=0.1350):
    """
    Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets
    
    Parameters:
        x: End effector x coordinate
        y: End effector y coordinate
        l1: Upper arm length (default 0.1159 m)
        l2: Lower arm length (default 0.1350 m)
        
    Returns:
        joint2, joint3: Joint angles in degrees as defined in the URDF file
    """

    # Calculate distance from origin to target point
    phi_wrist = math.sqrt(r**2 + z**2)
    phi_wrist_max = l1 + l2  # Maximum reachable distance
    
    # If target point is beyond maximum workspace, scale it to the boundary
    if phi_wrist > phi_wrist_max:
        print("greater than phi_wrist_max")
        scale_factor = phi_wrist_max / phi_wrist
        r *= scale_factor
        z *= scale_factor
        phi_wrist = phi_wrist_max
    
    # If target point is less than minimum workspace (|l1-l2|), scale it
    phi_wrist_min = abs(l1 - l2)
    if phi_wrist < phi_wrist_min and r > 0:
        print("less than r_min")
        scale_factor = phi_wrist_min / phi_wrist
        r *= scale_factor
        z *= scale_factor
        phi_wrist = phi_wrist_min
    
    # Use law of cosines to calculate theta2
    cos_theta_l = -(phi_wrist**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    # Calculate theta2 (elbow angle)
    theta_l = math.acos(cos_theta_l)
    joint3 = math.pi / 2 - theta_l

    # Calculate theta1 (shoulder angle)
    beta = math.atan2(z, r)

    cos_theta_b = -(l2**2 - l1**2 - phi_wrist**2) / (2 * l1 * phi_wrist)
    theta_b = math.acos(cos_theta_b)

    joint2 = -1 * (theta_b + beta - math.pi / 2)

    # we shouldn't need to do bounds checking because the motor bus class will enforce that the robot
    # will never leave the range of motion (mins and maxes) defined during calibration.  
    
    # Convert from radians to degrees
    joint2_deg = math.degrees(joint2)
    joint3_deg = math.degrees(joint3)

    return joint2_deg, joint3_deg


def ee_to_wrist_frame(robot: Robot, x, y, z, l3):
# transfer x, y, z from the end effector frame to the frame of the wrist joint
    curr_obs = robot.get_observation()
    curr_t1 = math.radians(curr_obs["arm_shoulder_pan.pos"])
    curr_t2 = math.radians(curr_obs["arm_shoulder_lift.pos"])
    curr_t3 = math.radians(curr_obs["arm_elbow_flex.pos"])
    curr_t4 = math.radians(curr_obs["arm_wrist_flex.pos"])

    curr_pitch = -1 * (curr_t2 + curr_t3 + curr_t4)

    # adjust vector is from the wrist to the end effector
    x_adjust = l3 * math.cos(-1 * curr_t1) * math.cos(curr_pitch)
    y_adjust = l3 * math.sin(-1 * curr_t1) * math.cos(curr_pitch) # we define +y as where theta 1 is negative.
    z_adjust = l3 * math.sin(curr_pitch)

    wrist_to_ee = (x_adjust, y_adjust, z_adjust)
    print(wrist_to_ee)

    # so, the adjust vector is subtracted from the current end effector point.
    x -= x_adjust
    y -= y_adjust
    z -= z_adjust

    return x, y, z


def xyz_inverse_kinematics(robot: Robot, x, y, z, l1=0.1159, l2=0.1375, l3=0.175):
    """
    Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets
    
    Parameters:
        x: End effector x coordinate (forward from the base of the arm is positive)
        y: End effector y coordinate (positive y is to the arm's left (from the arm's perspective))
        z: End effector z coordiante
        l1: Upper arm length (default 0.1159 m)
        l2: Lower arm length (default 0.1375 m)
        l3: Wrist joint to end effector tip/control point length (default 0.175 m)
        
    Returns:
        joint1, joint2, joint3: Joint angles in degrees as defined in the URDF file
    """


    # Convert from the end effector's frame to the wrist's frame
    x, y, z = ee_to_wrist_frame(robot, x, y, z, l3)
    


    # start doing the inverse kinematics for joints 1-3
    joint1_deg = math.degrees(math.atan2(y, x))

    r = math.sqrt(x**2 + y**2)

    # Calculate distance from origin to target point
    phi_wrist = math.sqrt(r**2 + z**2)
    phi_wrist_max = l1 + l2  # Maximum reachable distance
    
    # If target point is beyond maximum workspace, scale it to the boundary
    if phi_wrist > phi_wrist_max:
        print("greater than phi_wrist_max")
        scale_factor = phi_wrist_max / phi_wrist
        r *= scale_factor
        z *= scale_factor
        phi_wrist = phi_wrist_max
    
    # If target point is less than minimum workspace (|l1-l2|), scale it
    phi_wrist_min = abs(l1 - l2)
    if phi_wrist < phi_wrist_min and r > 0:
        print("less than r_min")
        scale_factor = phi_wrist_min / phi_wrist
        r *= scale_factor
        z *= scale_factor
        phi_wrist = phi_wrist_min
    
    # Use law of cosines to calculate theta2
    cos_theta_l = -(phi_wrist**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    # Calculate theta2 (elbow angle)
    theta_l = math.acos(cos_theta_l)
    joint3 = math.pi / 2 - theta_l

    # Calculate theta1 (shoulder angle)
    beta = math.atan2(z, r)

    cos_theta_b = -(l2**2 - l1**2 - phi_wrist**2) / (2 * l1 * phi_wrist)
    theta_b = math.acos(cos_theta_b)

    joint2 = -1 * (theta_b + beta - math.pi / 2)

    # we shouldn't need to do bounds checking because the motor bus class will enforce that the robot
    # will never leave the range of motion (mins and maxes) defined during calibration.  
    
    # Convert from radians to degrees

    joint2_deg = math.degrees(joint2)
    joint3_deg = math.degrees(joint3)

    return joint1_deg, joint2_deg, joint3_deg




def xyzp_inverse_kinematics(robot: Robot, x0, y0, z0, pitch_rad, l1=0.1159, l2=0.1375, l3=0.195):
    """
    Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets
    
    Parameters:
        x: End effector x coordinate (forward from the base of the arm is positive)
        y: End effector y coordinate (positive y is to the arm's left (from the arm's perspective))
        z: End effector z coordiante
        l1: Upper arm length (default 0.1159 m)
        l2: Lower arm length (default 0.1375 m)
        l3: Wrist joint to end effector tip/control point length (default 0.195 m)
        
    Returns:
        ((success/fail, (joint1, joint2, joint3, joint4)): Joint angles in degrees as defined in the URDF file
    """


    curr_obs = read_angles(robot)
    curr_t2 = curr_obs["arm_shoulder_pan"] * math.pi / 180
    curr_t3 = curr_obs["arm_elbow_flex"] * math.pi / 180

    # Convert from the end effector's frame to the wrist's frame
    # x, y, z = ee_to_wrist_frame(robot, x, y, z, l3)
    
    # start doing the inverse kinematics for joints 1-3

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
            # numerical solver returned a bad solution (the robot cannot physicall reach it).
            # therefore, we say the IK solver failed.
            return (False, (0, 0, 0, 0)) 

    return (True, joints_degs)






    r = math.sqrt(x**2 + y**2)

    # Calculate distance from origin to target point
    phi_wrist = math.sqrt(r**2 + z**2)
    phi_wrist_max = l1 + l2  # Maximum reachable distance
    
    # If target point is beyond maximum workspace, scale it to the boundary
    if phi_wrist > phi_wrist_max:
        print("greater than phi_wrist_max")
        scale_factor = phi_wrist_max / phi_wrist
        r *= scale_factor
        z *= scale_factor
        phi_wrist = phi_wrist_max
    
    # If target point is less than minimum workspace (|l1-l2|), scale it
    phi_wrist_min = abs(l1 - l2)
    if phi_wrist < phi_wrist_min and r > 0:
        print("less than r_min")
        scale_factor = phi_wrist_min / phi_wrist
        r *= scale_factor
        z *= scale_factor
        phi_wrist = phi_wrist_min
    
    # Use law of cosines to calculate theta2
    cos_theta_l = -(phi_wrist**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    # Calculate theta2 (elbow angle)
    theta_l = math.acos(cos_theta_l)
    joint3 = math.pi / 2 - theta_l

    # Calculate theta1 (shoulder angle)
    beta = math.atan2(z, r)

    cos_theta_b = -(l2**2 - l1**2 - phi_wrist**2) / (2 * l1 * phi_wrist)
    theta_b = math.acos(cos_theta_b)

    joint2 = -1 * (theta_b + beta - math.pi / 2)

    # we shouldn't need to do bounds checking because the motor bus class will enforce that the robot
    # will never leave the range of motion (mins and maxes) defined during calibration.  
    
    # Convert from radians to degrees

    joint2_deg = math.degrees(joint2)
    joint3_deg = math.degrees(joint3)

    return joint1_deg, joint2_deg, joint3_deg


def move_to_zero_position(robot, duration=3.0, kp=0.5):
    """
    Use P control to slowly move robot to zero position
    
    Args:
        robot: robot instance
        duration: time to move to zero position (seconds)
        kp: proportional gain
    """
    print("Using P control to slowly move robot to zero position...")
    
    # Get current robot state
    current_obs = robot.get_observation()
    
    # Extract current joint positions
    current_positions = {}
    for key, value in current_obs.items():
        if key.endswith('.pos'):
            motor_name = key.removesuffix('.pos')
            current_positions[motor_name] = value
    
    # Zero position targets
    zero_positions = {
        'arm_shoulder_pan': 0.0,
        'arm_shoulder_lift': 0.0,
        'arm_elbow_flex': 0.0,
        'arm_wrist_flex': 0.0,
        'arm_wrist_roll': 0.0,
        'arm_gripper': 0.0
    }
    
    # Calculate control steps
    control_freq = 50  # 50Hz control frequency
    total_steps = int(duration * control_freq)
    step_time = 1.0 / control_freq
    
    print(f"Will use P control to move to zero position in {duration} seconds, control frequency: {control_freq}Hz, proportional gain: {kp}")
    
    for step in range(total_steps):
        # Get current robot state
        current_obs = robot.get_observation()
        current_positions = {}
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                # Apply calibration coefficients
                calibrated_value = apply_joint_calibration(motor_name, value)
                current_positions[motor_name] = calibrated_value
        
        # P control calculation
        robot_action = {}
        for joint_name, target_pos in zero_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                
                # P control: output = Kp * error
                control_output = kp * error
                
                # Convert control output to position command
                new_position = current_pos + control_output
                robot_action[f"{joint_name}.pos"] = new_position
        
        # Send action to robot
        if robot_action:
            robot_action["x.vel"] = 0.0
            robot_action["y.vel"] = 0.0
            robot_action["theta.vel"] = 0.0

            robot.send_action(robot_action)
        
        # Show progress
        if step % (control_freq // 2) == 0:  # Show progress every 0.5 seconds
            progress = (step / total_steps) * 100
            print(f"Moving to zero position progress: {progress:.1f}%")
        
        time.sleep(step_time)
    
    print("Robot has moved to zero position")

def return_to_start_position(robot, start_positions, kp=0.5, control_freq=50):
    """
    Use P control to return to start position
    
    Args:
        robot: robot instance
        start_positions: start joint position dictionary
        kp: proportional gain
        control_freq: control frequency (Hz)
    """
    print("Returning to start position...")
    
    control_period = 1.0 / control_freq
    max_steps = int(5.0 * control_freq)  # Maximum 5 seconds
    
    for step in range(max_steps):
        # Get current robot state
        current_obs = robot.get_observation()
        current_positions = {}
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                current_positions[motor_name] = value  # Don't apply calibration coefficients
        
        # P control calculation
        robot_action = {}
        total_error = 0
        for joint_name, target_pos in start_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                total_error += abs(error)
                
                # P control: output = Kp * error
                control_output = kp * error
                
                # Convert control output to position command
                new_position = current_pos + control_output
                robot_action[f"{joint_name}.pos"] = new_position
        
        # Send action to robot
        if robot_action:
            robot_action["x.vel"] = 0.0
            robot_action["y.vel"] = 0.0
            robot_action["theta.vel"] = 0.0

            # print("commanding robot to send action:\n", robot_action)
            robot.send_action(robot_action)
        
        # Check if reached start position
        print("Total error is, ", total_error)
        if total_error < 2.0:  # If total error is less than 2 degrees, consider reached
            print("Returned to start position")
            break
        
        time.sleep(control_period)
    
    print("Return to start position completed")

# def p_control_loop(robot, keyboard, target_positions, start_positions, current_x, current_y, kp=0.5, control_freq=50):
#     """
#     P control loop
    
#     Args:
#         robot: robot instance
#         keyboard: keyboard instance
#         target_positions: target joint position dictionary
#         start_positions: start joint position dictionary
#         current_x: current x coordinate (distance "forward" from the base of the robot)
#         current_y: current y coordinate (left/right distance, left from the robot's perspective is positive)
#         current_z: current z coordiante (z=0 is the same height as the 2nd motor)
#         kp: proportional gain
#         control_freq: control frequency (Hz)
#     """
#     control_period = 1.0 / control_freq
    
#     # Initialize pitch control variables
#     pitch = 0.0  # Initial pitch adjustment
#     pitch_step = 1  # Pitch adjustment step size
    
#     print(f"Starting P control loop, control frequency: {control_freq}Hz, proportional gain: {kp}")
    
#     while True:
#         try:
#             # Get keyboard input
#             keyboard_action = keyboard.get_action()
            
#             if keyboard_action:
#                 # Process keyboard input, update target positions
#                 for key, value in keyboard_action.items():
#                     if key == 'x':
#                         # Exit program, first return to start position
#                         print("Exit command detected, returning to start position...")
#                         return_to_start_position(robot, start_positions, 0.2, control_freq)
#                         return
                    
#                     # Joint control mapping
#                     joint_controls = {
#                         'q': ('arm_shoulder_pan', -1),    # Joint 1 decrease
#                         'a': ('arm_shoulder_pan', 1),     # Joint 1 increase
#                         't': ('arm_wrist_roll', -1),      # Joint 5 decrease
#                         'g': ('arm_wrist_roll', 1),       # Joint 5 increase
#                         'y': ('arm_gripper', -1),         # Joint 6 decrease
#                         'h': ('arm_gripper', 1),          # Joint 6 increase
#                     }
                    
#                     # x,y coordinate control
#                     xy_controls = {
#                         'w': ('x', -0.004),  # x decrease
#                         's': ('x', 0.004),   # x increase
#                         'e': ('y', -0.004),  # y decrease
#                         'd': ('y', 0.004),   # y increase
#                     }
                    
#                     # Pitch control
#                     if key == 'r':
#                         pitch += pitch_step
#                         print(f"Increase pitch adjustment: {pitch:.3f}")
#                     elif key == 'f':
#                         pitch -= pitch_step
#                         print(f"Decrease pitch adjustment: {pitch:.3f}")
                    
#                     if key in joint_controls:
#                         joint_name, delta = joint_controls[key]
#                         if joint_name in target_positions:
#                             current_target = target_positions[joint_name]
#                             new_target = int(current_target + delta)
#                             target_positions[joint_name] = new_target
#                             print(f"Update target position {joint_name}: {current_target} -> {new_target}")
                    
#                     elif key in xy_controls:
#                         coord, delta = xy_controls[key]
#                         if coord == 'x':
#                             current_x += delta
#                             if (ik_check_in_bounds(current_x, current_y) == True):
#                                 # we're going to go out of range. Don't proceed. 
#                                 # Calculate target angles for joint2 and joint3
#                                 joint1_target, joint2_target, joint3_target = xyz_inverse_kinematics(robot, current_x, current_y)
#                                 target_positions['arm_shoulder_pan'] = joint1_target
#                                 target_positions['arm_shoulder_lift'] = joint2_target
#                                 target_positions['arm_elbow_flex'] = joint3_target
#                                 print(f"Update x coordinate: {current_x:.4f}, joint2={joint2_target:.3f}, joint3={joint3_target:.3f}")
#                             else:
#                                 print("would leave range")
#                                 current_x -= delta
#                         elif coord == 'y':
#                             current_y += delta
#                             if (ik_check_in_bounds(current_x, current_y) == True):
#                                 # Calculate target angles for joint2 and joint3
#                                 joint1_target, joint2_target, joint3_target = xyz_inverse_kinematics(current_x, current_y)
#                                 target_positions['arm_shoulder_pan'] = joint1_target
#                                 target_positions['arm_shoulder_lift'] = joint2_target
#                                 target_positions['arm_elbow_flex'] = joint3_target
#                                 print(f"Update y coordinate: {current_y:.4f}, joint2={joint2_target:.3f}, joint3={joint3_target:.3f}")
#                             else:
#                                 print("would leave range")
#                                 current_y -= delta
            
#             # Apply pitch adjustment to arm_wrist_flex
#             # Calculate arm_wrist_flex target position based on arm_shoulder_lift and arm_elbow_flex
#             if 'arm_shoulder_lift' in target_positions and 'arm_elbow_flex' in target_positions:
#                 target_positions['arm_wrist_flex'] = - target_positions['arm_shoulder_lift'] - target_positions['arm_elbow_flex'] + pitch
#                 # Show current pitch value (display every 100 steps to avoid screen flooding)
#                 if hasattr(p_control_loop, 'step_counter'):
#                     p_control_loop.step_counter += 1
#                 else:
#                     p_control_loop.step_counter = 0
                
#                 if p_control_loop.step_counter % 100 == 0:
#                     print(f"Current pitch adjustment: {pitch:.3f}, arm_wrist_flex target: {target_positions['arm_wrist_flex']:.3f}")
            
#             # Get current robot state
#             current_obs = robot.get_observation()
            
#             # Extract current joint positions
#             current_positions = {}
#             for key, value in current_obs.items():
#                 if key.endswith('.pos'):
#                     motor_name = key.removesuffix('.pos')
#                     # Apply calibration coefficients
#                     calibrated_value = apply_joint_calibration(motor_name, value)
#                     current_positions[motor_name] = calibrated_value
            
#             # P control calculation
#             robot_action = {}
#             for joint_name, target_pos in target_positions.items():
#                 if joint_name in current_positions:
#                     current_pos = current_positions[joint_name]
#                     error = target_pos - current_pos
                    
#                     # P control: output = Kp * error
#                     control_output = kp * error
                    
#                     # Convert control output to position command
#                     new_position = current_pos + control_output
#                     robot_action[f"{joint_name}.pos"] = new_position
            
#             # Send action to robot
#             if robot_action:
#                 robot_action["x.vel"] = 0.0
#                 robot_action["y.vel"] = 0.0
#                 robot_action["theta.vel"] = 0.0

#                 # print("was going to send action", robot_action)
#                 robot.send_action(robot_action)
            
#             time.sleep(control_period)
            
#         except KeyboardInterrupt:
#             print("User interrupted program")
#             break
#         except Exception as e:
#             print(f"P control loop error: {e}")
#             traceback.print_exc()
#             break




def p_control_loop_2(robot: Robot, keyboard: KeyboardTeleop, target_positions, start_positions, xyz_start_pos, kp=0.5, control_freq=50):
    """
    P control loop
    
    Args:
        robot: robot instance
        keyboard: keyboard instance
        target_positions: target joint position dictionary
        start_positions: start joint position dictionary
        xyz_start_pos: a tuple containing:
            current_x: current x coordinate (distance "forward" from the base of the robot),
            current_y: current y coordinate (left/right distance, left from the robot's perspective is positive),
            current_z: current z coordiante (z=0 is the same height as the 2nd motor)
        kp: proportional gain
        control_freq: control frequency (Hz)
    """
    current_xyz = {"x" : xyz_start_pos[0], "y" : xyz_start_pos[1], "z" : xyz_start_pos[2]}
    xyz_ranges = {"x" : {"min" : 0.01, "max" : 0.45}, "y" : {"min" : -0.45, "max" : 0.45}, "z" : {"min" : -0.4, "max" : 0.45}}


    control_period = 1.0 / control_freq
    # control_period = 0.5
    
    # Initialize pitch control variables
    pitch = 0.0  # Initial pitch adjustment
    pitch_deg_per_sec = 50 # pitch moves at 20 degrees per second (assuming cycles are instant, which they aren't)
    xyz_L1_m_per_sec = 0.2 # speed per direction

    print(f"Starting P control loop, control frequency: {control_freq}Hz, proportional gain: {kp}")
    
    while True:
        try:
            # Get keyboard input
            keyboard_action = keyboard.get_action()
            
            if keyboard_action:
                # Process keyboard input, update target positions
                changed_xyzp = False
                for key in keyboard_action.keys():
                    if key == 'x':
                        # Exit program, first return to start position
                        print("Exit command detected, returning to start position...")
                        return_to_start_position(robot, start_positions, 0.2, control_freq)
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
                            current_target = target_positions[joint_name]
                            new_target = int(current_target + delta)
                            target_positions[joint_name] = new_target
                            print(f"Update target position {joint_name}: {current_target} -> {new_target}")
                    
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

                # print("was going to send action", robot_action)
                print(current_xyz, pitch * 180 / math.pi)
                robot.send_action(robot_action)
            
            time.sleep(control_period)
            
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

    FPS = 30

    # Create the robot and teleoperator configurations
    robot_config = LeKiwiClientConfig(remote_ip="192.168.0.133", id="my_lekiwi2")
    teleop_arm_config = SO100LeaderConfig(port="COM5", id="my_awesome_leader_arm")
    keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

    # LeKiwiConfig(use_degrees=True)

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


    # from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
    # my_follower = SO101Follower(SO101FollowerConfig(port="hi", use_degrees=True))


    # my_follower.bus.sync_read("Present_Position")

    mode_dict = {0 : "ik", 1 : "print ctrl", 2 : "normal", 3 : "zero"}
    mode = mode_dict[0]

    # IK control code start 
    if (mode == "ik"):
        try:
            read_and_print_angles(robot)
            
            # Move to zero position

            # I have to hack the zero position a bit for the robot to overcome its own gravity and actually come to rest at zero (after send action)
            zero_action = {'arm_shoulder_pan.pos': 0.0, 'arm_shoulder_lift.pos': -2.0, 'arm_elbow_flex.pos': -5.0, 'arm_wrist_flex.pos': 0.0, 'arm_wrist_roll.pos': 0.0, 'arm_gripper.pos': 11.0, 'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
            robot.send_action(zero_action)

            busy_wait(3)

            start_positions = read_and_print_angles(robot)


            # Initialize target positions as current positions (integers)


            zero_poses = {
            'arm_shoulder_pan': 0.0,
            'arm_shoulder_lift': 0.0,
            'arm_elbow_flex': 0.0,
            'arm_wrist_flex': 0.0,
            'arm_wrist_roll': 0.0,
            'arm_gripper': 100.0
            }
            
            # Initialize x,y coordinate control
            x0, y0, z0 = 0.1375 + 0.195, 0.0, 0.1159  # x0 = length of elbow to end effector (arm pointing straight out), z0 = length of shoulder to elbow (pointing straight up). Robot starts at all (non-gripper) angles = 0
            print(f"Initialize end effector position: x={x0:.4f}, y={y0:.4f}, z={z0:.4f}")
            
            
            print("Keyboard control instructions:")
            # print("- Q/A: Joint 1 (arm_shoulder_pan) decrease/increase")
            # print("- W/S: Control end effector r coordinate (joint2+3)")
            # print("- E/D: Control end effector z coordinate (joint2+3)")
            print("- W/S: X coordinate (forward/backward)")
            print("- A/D: Y coordinate change (left/right)")
            print("- Q/E: Z coordinate change (up/down)")
            print("- R/F: Pitch adjustment increase/decrease (affects arm_wrist_flex)")
            print("- T/G: Joint 5 (arm_wrist_roll) decrease/increase")
            print("- Y/H: Joint 6 (arm_gripper) decrease/increase")
            print("- X: Exit program (return to start position first)")
            print("- ESC: Exit program")
            print("="*50)
            print("Note: Robot will continuously move to target positions")
            
            # Start P control loop
            # third slot is the initial target positiosn for all the joints that aren't being controlled by inverse kinematics.
            p_control_loop_2(robot, keyboard, zero_poses, start_positions, (x0, y0, z0), kp=0.5, control_freq=50)
            
            # r0, z0 = 0.15, 0.1159 

            # j2, j3 = inverse_kinematics(r0, z0)
            # print("Joint 2 angle ori:", j2)
            # print("Joint 3 angle ori:", j3)

            # j2, j3 = inverse_kinematics_TWO(r0, z0)
            # print("Joint 2 angle NEW:", j2)
            # print("Joint 3 angle NEW:", j3)
            # expect 0, 0

            # x0, y0, z0 = 0.1375, 0.0, 0.1159
            # print(xyz_inverse_kinematics(robot, x0, y0, z0))


            # print(xyzp_inverse_kinematics(robot, 0.25, 0, 0.11, 0))


            # Disconnect
            robot.disconnect()
            keyboard.disconnect()
            print("Program ended")
            pass

        except Exception as e:
            print(f"Program execution failed: {e}")
            traceback.print_exc()
            print("Please check:")
            print("1. Whether the robot is properly connected")
            print("2. Whether the USB port is correct")
            print("3. Whether you have sufficient permissions to access USB devices")
            print("4. Whether the robot is properly configured")



    if (mode == "zero"):

        # move_to_zero_position(robot)

        zero_action = {'arm_shoulder_pan.pos': 0.0, 'arm_shoulder_lift.pos': 0.0, 'arm_elbow_flex.pos': 0.0, 'arm_wrist_flex.pos': 0.0, 'arm_wrist_roll.pos': 0.0, 'arm_gripper.pos': 11.0, 'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
        robot.send_action(zero_action)

        busy_wait(3)

        observation = robot.get_observation()

        
        print("Reading joint angles...")
        start_positions = {}
        for key, value in observation.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                start_positions[motor_name] = int(value)  # Don't apply calibration coefficients
        
        print("Joint angles:")
        for joint_name, position in start_positions.items():
            print(f"  {joint_name}: {position}°")   

        return



    if (mode == "print ctrl"):
    # set configuration control pritning out motor angles
        while True:
            t0 = time.perf_counter()

            observation = robot.get_observation()

            
            print("Reading joint angles...")
            start_positions = {}
            for key, value in observation.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    start_positions[motor_name] = int(value)  # Don't apply calibration coefficients
            
            print("Joint angles:")
            for joint_name, position in start_positions.items():
                print(f"  {joint_name}: {position}°")



            # new_pos = {"arm_shoulder_pan": ,
            #             "arm_shoulder_lift": ,
            #             "arm_elbow_flex": ,
            #             "arm_wrist_flex": ,
            #             "arm_wrist_roll": ,
            #             "arm_gripper": }
            
            # resting_pos = {'arm_shoulder_pan': , 'arm_shoulder_lift': , 'arm_elbow_flex': , 'arm_wrist_flex': , 'arm_wrist_roll': , 'arm_gripper': }


            # return_to_start_position(robot, new_pos, 0.5, control_freq=50)


    

            arm_action = leader_arm.get_action()
            arm_action = {f"arm_{k}": v for k, v in arm_action.items()}

            keyboard_keys = keyboard.get_action()
            base_action = robot._from_keyboard_to_base_action(keyboard_keys)

            log_rerun_data(observation, {**arm_action, **base_action})

            action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

            robot.send_action(action)

            # busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
            busy_wait(1)




    if (mode == "normal"):
    # default control via leader arm 
        while True:
            t0 = time.perf_counter()

            observation = robot.get_observation()

            arm_action = leader_arm.get_action()
            arm_action = {f"arm_{k}": v for k, v in arm_action.items()}

            keyboard_keys = keyboard.get_action()
            base_action = robot._from_keyboard_to_base_action(keyboard_keys)

            log_rerun_data(observation, {**arm_action, **base_action})

            action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

            # print(action)

            robot.send_action(action)

            # busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
            busy_wait(0.25)


if (__name__ == "__main__"):
    main()