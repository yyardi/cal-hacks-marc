import time

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import _init_rerun, log_rerun_data


import time
import logging
import traceback
import math

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Joint calibration coefficients - manually edited
# Format: [joint_name, zero_position_offset(degrees), scale_factor]
JOINT_CALIBRATION = [
    ['arm_shoulder_pan', 6.0, 1.0],      # Joint 1: zero position offset, scale factor
    ['arm_shoulder_lift', 2.0, 0.97],     # Joint 2: zero position offset, scale factor
    ['arm_elbow_flex', 0.0, 1.05],        # Joint 3: zero position offset, scale factor
    ['arm_wrist_flex', 0.0, 0.94],        # Joint 4: zero position offset, scale factor
    ['arm_wrist_roll', 0.0, 0.5],        # Joint 5: zero position offset, scale factor
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

def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    """
    Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets
    
    Parameters:
        x: End effector x coordinate
        y: End effector y coordinate
        l1: Upper arm length (default 0.1159 m)
        l2: Lower arm length (default 0.1350 m)
        
    Returns:
        joint2, joint3: Joint angles in radians as defined in the URDF file
    """
    # Calculate joint2 and joint3 offsets in theta1 and theta2
    theta1_offset = math.atan2(0.028, 0.11257)  # theta1 offset when joint2=0
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset  # theta2 offset when joint3=0
    
    # Calculate distance from origin to target point
    r = math.sqrt(x**2 + y**2)
    r_max = l1 + l2  # Maximum reachable distance
    
    # If target point is beyond maximum workspace, scale it to the boundary
    if r > r_max:
        scale_factor = r_max / r
        x *= scale_factor
        y *= scale_factor
        r = r_max
    
    # If target point is less than minimum workspace (|l1-l2|), scale it
    r_min = abs(l1 - l2)
    if r < r_min and r > 0:
        scale_factor = r_min / r
        x *= scale_factor
        y *= scale_factor
        r = r_min
    
    # Use law of cosines to calculate theta2
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    # Calculate theta2 (elbow angle)
    theta2 = math.pi - math.acos(cos_theta2)
    
    # Calculate theta1 (shoulder angle)
    beta = math.atan2(y, x)
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

def p_control_loop(robot, keyboard, target_positions, start_positions, current_x, current_y, kp=0.5, control_freq=50):
    """
    P control loop
    
    Args:
        robot: robot instance
        keyboard: keyboard instance
        target_positions: target joint position dictionary
        start_positions: start joint position dictionary
        current_x: current x coordinate
        current_y: current y coordinate
        kp: proportional gain
        control_freq: control frequency (Hz)
    """
    control_period = 1.0 / control_freq
    control_period = 5
    
    # Initialize pitch control variables
    pitch = 0.0  # Initial pitch adjustment
    pitch_step = 1  # Pitch adjustment step size
    
    print(f"Starting P control loop, control frequency: {control_freq}Hz, proportional gain: {kp}")
    
    while True:
        try:
            # Get keyboard input
            keyboard_action = keyboard.get_action()
            
            if keyboard_action:
                # Process keyboard input, update target positions
                for key, value in keyboard_action.items():
                    if key == 'x':
                        # Exit program, first return to start position
                        print("Exit command detected, returning to start position...")
                        return_to_start_position(robot, start_positions, 0.2, control_freq)
                        return
                    
                    # Joint control mapping
                    joint_controls = {
                        'q': ('arm_shoulder_pan', -1),    # Joint 1 decrease
                        'a': ('arm_shoulder_pan', 1),     # Joint 1 increase
                        't': ('arm_wrist_roll', -1),      # Joint 5 decrease
                        'g': ('arm_wrist_roll', 1),       # Joint 5 increase
                        'y': ('arm_gripper', -1),         # Joint 6 decrease
                        'h': ('arm_gripper', 1),          # Joint 6 increase
                    }
                    
                    # x,y coordinate control
                    xy_controls = {
                        'w': ('x', -0.004),  # x decrease
                        's': ('x', 0.004),   # x increase
                        'e': ('y', -0.004),  # y decrease
                        'd': ('y', 0.004),   # y increase
                    }
                    
                    # Pitch control
                    if key == 'r':
                        pitch += pitch_step
                        print(f"Increase pitch adjustment: {pitch:.3f}")
                    elif key == 'f':
                        pitch -= pitch_step
                        print(f"Decrease pitch adjustment: {pitch:.3f}")
                    
                    if key in joint_controls:
                        joint_name, delta = joint_controls[key]
                        if joint_name in target_positions:
                            current_target = target_positions[joint_name]
                            new_target = int(current_target + delta)
                            target_positions[joint_name] = new_target
                            print(f"Update target position {joint_name}: {current_target} -> {new_target}")
                    
                    elif key in xy_controls:
                        coord, delta = xy_controls[key]
                        if coord == 'x':
                            current_x += delta
                            # Calculate target angles for joint2 and joint3
                            joint2_target, joint3_target = inverse_kinematics(current_x, current_y)
                            target_positions['arm_shoulder_lift'] = joint2_target
                            target_positions['arm_elbow_flex'] = joint3_target
                            print(f"Update x coordinate: {current_x:.4f}, joint2={joint2_target:.3f}, joint3={joint3_target:.3f}")
                        elif coord == 'y':
                            current_y += delta
                            # Calculate target angles for joint2 and joint3
                            joint2_target, joint3_target = inverse_kinematics(current_x, current_y)
                            target_positions['arm_shoulder_lift'] = joint2_target
                            target_positions['arm_elbow_flex'] = joint3_target
                            print(f"Update y coordinate: {current_y:.4f}, joint2={joint2_target:.3f}, joint3={joint3_target:.3f}")
            
            # Apply pitch adjustment to arm_wrist_flex
            # Calculate arm_wrist_flex target position based on arm_shoulder_lift and arm_elbow_flex
            if 'arm_shoulder_lift' in target_positions and 'arm_elbow_flex' in target_positions:
                target_positions['arm_wrist_flex'] = - target_positions['arm_shoulder_lift'] - target_positions['arm_elbow_flex'] + pitch
                # Show current pitch value (display every 100 steps to avoid screen flooding)
                if hasattr(p_control_loop, 'step_counter'):
                    p_control_loop.step_counter += 1
                else:
                    p_control_loop.step_counter = 0
                
                if p_control_loop.step_counter % 100 == 0:
                    print(f"Current pitch adjustment: {pitch:.3f}, arm_wrist_flex target: {target_positions['arm_wrist_flex']:.3f}")
            
            # Get current robot state
            current_obs = robot.get_observation()
            
            # Extract current joint positions
            current_positions = {}
            for key, value in current_obs.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    # Apply calibration coefficients
                    calibrated_value = apply_joint_calibration(motor_name, value)
                    current_positions[motor_name] = calibrated_value
            
            # P control calculation
            robot_action = {}
            for joint_name, target_pos in target_positions.items():
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

                print("was going to send action", robot_action)
                # robot.send_action(robot_action)
            
            time.sleep(control_period)
            
        except KeyboardInterrupt:
            print("User interrupted program")
            break
        except Exception as e:
            print(f"P control loop error: {e}")
            traceback.print_exc()
            break



def main():
    # TODO: Assemble the kiwi, configure it if necessary, set the port for the leader arm if using that, and continue 

    FPS = 30

    # Create the robot and teleoperator configurations
    robot_config = LeKiwiClientConfig(remote_ip="172.18.134.136", id="my_lekiwi")
    teleop_arm_config = SO100LeaderConfig(port="/dev/tty.usbmodem585A0077581", id="my_awesome_leader_arm")
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
        # Read initial joint angles
        print("Reading initial joint angles...")
        start_obs = robot.get_observation()
        start_positions = {}
        for key, value in start_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                start_positions[motor_name] = int(value)  # Don't apply calibration coefficients
        
        print("Initial joint angles:")
        for joint_name, position in start_positions.items():
            print(f"  {joint_name}: {position}°")
        
        # Move to zero position

        # move_to_zero_position(robot)


        # Initialize target positions as current positions (integers)
        
        # target_positions = {"arm_shoulder_pan": 0.0,
        #             "arm_shoulder_lift": 0.0,
        #             "arm_elbow_flex": 0.0,
        #             "arm_wrist_flex": 0.0,
        #             "arm_wrist_roll": 0.0,
        #             "arm_gripper": 0.0}

        # return_to_start_position(robot, target_positions, 0.5, control_freq=50)


        # target_positions = {
        # 'arm_shoulder_pan': 0.0,
        # 'arm_shoulder_lift': 0.0,
        # 'arm_elbow_flex': 0.0,
        # 'arm_wrist_flex': 0.0,
        # 'arm_wrist_roll': 0.0,
        # 'arm_gripper': 0.0
        #   }
        
        # Initialize x,y coordinate control
        x0, y0 = 0.1629, 0.1131
        current_x, current_y = x0, y0
        print(f"Initialize end effector position: x={current_x:.4f}, y={current_y:.4f}")
        
        
        print("Keyboard control instructions:")
        print("- Q/A: Joint 1 (arm_shoulder_pan) decrease/increase")
        print("- W/S: Control end effector x coordinate (joint2+3)")
        print("- E/D: Control end effector y coordinate (joint2+3)")
        print("- R/F: Pitch adjustment increase/decrease (affects arm_wrist_flex)")
        print("- T/G: Joint 5 (arm_wrist_roll) decrease/increase")
        print("- Y/H: Joint 6 (arm_gripper) decrease/increase")
        print("- X: Exit program (return to start position first)")
        print("- ESC: Exit program")
        print("="*50)
        print("Note: Robot will continuously move to target positions")
        
        # Start P control loop
        # p_control_loop(robot, keyboard, target_positions, start_positions, current_x, current_y, kp=0.5, control_freq=50)
        
        print(inverse_kinematics(0.05, 0.05)) # want pos neg


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




    # set configuration control pritning out motor angles
    # while True:
    #     t0 = time.perf_counter()

    #     observation = robot.get_observation()

        
    #     print("Reading joint angles...")
    #     start_positions = {}
    #     for key, value in observation.items():
    #         if key.endswith('.pos'):
    #             motor_name = key.removesuffix('.pos')
    #             start_positions[motor_name] = int(value)  # Don't apply calibration coefficients
        
    #     print("Joint angles:")
    #     for joint_name, position in start_positions.items():
    #         print(f"  {joint_name}: {position}°")



    #     # new_pos = {"arm_shoulder_pan": ,
    #     #             "arm_shoulder_lift": ,
    #     #             "arm_elbow_flex": ,
    #     #             "arm_wrist_flex": ,
    #     #             "arm_wrist_roll": ,
    #     #             "arm_gripper": }
        
    #     # resting_pos = {'arm_shoulder_pan': , 'arm_shoulder_lift': , 'arm_elbow_flex': , 'arm_wrist_flex': , 'arm_wrist_roll': , 'arm_gripper': }


    #     # return_to_start_position(robot, new_pos, 0.5, control_freq=50)


  

    #     arm_action = leader_arm.get_action()
    #     arm_action = {f"arm_{k}": v for k, v in arm_action.items()}

    #     keyboard_keys = keyboard.get_action()
    #     base_action = robot._from_keyboard_to_base_action(keyboard_keys)

    #     log_rerun_data(observation, {**arm_action, **base_action})

    #     action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

    #     robot.send_action(action)

    #     busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))




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

        robot.send_action(action)

        busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))


if (__name__ == "__main__"):
    main()