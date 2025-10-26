#!/usr/bin/env python

"""
Improved teleoperation with better IK failure handling and diagnostics
Key improvements:
1. Verbose IK failure reporting
2. Recovery mechanisms when IK fails
3. Position validation before sending commands
4. Communication verification
5. Logging for debugging
"""

import time
import traceback
import math
from sympy import cos, sin, nsolve
from sympy.abc import x, y

# Add logging
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class IKFailureError(Exception):
    """Custom exception for IK failures"""
    pass


def xyzp_inverse_kinematics_improved(robot, x0, y0, z0, pitch_rad, l1=0.1159, l2=0.1375, l3=0.17, verbose=True):
    """
    Improved IK with better error handling and diagnostics
    
    Returns:
        (success: bool, joints: tuple, error_msg: str)
    """
    
    # Validate input ranges first
    max_reach = l1 + l2 + l3
    r = math.sqrt(x0**2 + y0**2)
    
    if r > max_reach:
        error_msg = f"Target distance {r:.3f}m exceeds max reach {max_reach:.3f}m"
        if verbose:
            logger.error(error_msg)
        return (False, (0, 0, 0, 0), error_msg)
    
    # Check Z limits
    max_z = l1 + l2 + l3
    min_z = -(l1 + l2 + l3)
    if z0 > max_z or z0 < min_z:
        error_msg = f"Z={z0:.3f} out of range [{min_z:.3f}, {max_z:.3f}]"
        if verbose:
            logger.error(error_msg)
        return (False, (0, 0, 0, 0), error_msg)
    
    # Get current angles for starting guess
    try:
        curr_obs = read_angles(robot)
        curr_t2 = curr_obs["shoulder_lift"] * math.pi / 180
        curr_t3 = curr_obs["elbow_flex"] * math.pi / 180
    except Exception as e:
        error_msg = f"Failed to read current angles: {e}"
        logger.error(error_msg)
        return (False, (0, 0, 0, 0), error_msg)
    
    # Try numerical solve with multiple starting points if first attempt fails
    starting_guesses = [
        [curr_t2, curr_t3],  # Current position
        [0, 0],              # Zero position
        [-math.pi/4, math.pi/4],  # Alternative guess
    ]
    
    solve_error = None
    for guess_idx, guess in enumerate(starting_guesses):
        try:
            res = nsolve(
                [l1 * sin(x) + l2 * cos(x + y) + l3 * cos(pitch_rad) - r, 
                 l1 * cos(x) - l2 * sin(x + y) + l3 * sin(pitch_rad) - z0], 
                [x, y], 
                guess, 
                prec=5
            )
            
            joint1_deg = math.degrees(math.atan2(y0, x0))
            joint2_deg = math.degrees(res[0])
            joint3_deg = math.degrees(res[1])
            joint4_deg = -1 * (joint2_deg + joint3_deg + (pitch_rad * 180 / math.pi))
            
            joints_degs = (joint1_deg, joint2_deg, joint3_deg, joint4_deg)
            
            # Check joint limits
            THETA_MAX = 100
            THETA_MIN = -100
            
            out_of_bounds = []
            for i, (joint, angle) in enumerate(zip(['pan', 'lift', 'flex', 'wrist'], joints_degs)):
                if angle > THETA_MAX or angle < THETA_MIN:
                    out_of_bounds.append(f"{joint}={angle:.1f}°")
            
            if out_of_bounds:
                error_msg = f"Joints out of bounds: {', '.join(out_of_bounds)}"
                if guess_idx < len(starting_guesses) - 1:
                    logger.warning(f"{error_msg} - trying alternate guess")
                    continue
                else:
                    if verbose:
                        logger.error(error_msg)
                    return (False, (0, 0, 0, 0), error_msg)
            
            # Success!
            if verbose and guess_idx > 0:
                logger.info(f"IK converged with starting guess #{guess_idx}")
            return (True, joints_degs, "")
            
        except Exception as e:
            solve_error = str(e)
            if guess_idx < len(starting_guesses) - 1:
                logger.debug(f"Solver failed with guess #{guess_idx}, trying next guess")
                continue
    
    # All attempts failed
    error_msg = f"IK solver failed after {len(starting_guesses)} attempts. Last error: {solve_error}"
    if verbose:
        logger.error(error_msg)
    return (False, (0, 0, 0, 0), error_msg)


def read_angles(robot):
    """Read current joint angles from robot"""
    start_obs = robot.get_observation()
    start_positions = {}
    for key, value in start_obs.items():
        if key.endswith('.pos'):
            motor_name = key.removesuffix('.pos')
            start_positions[motor_name] = value
    return start_positions


def verify_robot_response(robot, expected_action, tolerance=5.0, timeout=2.0):
    """
    Verify that robot actually moved to expected position
    
    Args:
        robot: Robot instance
        expected_action: Dictionary of expected positions
        tolerance: Acceptable error in degrees
        timeout: How long to wait for robot to reach position
        
    Returns:
        (success: bool, error_msg: str)
    """
    time.sleep(timeout)
    
    current_pos = read_angles(robot)
    errors = []
    
    for joint, expected in expected_action.items():
        if not joint.endswith('_pos'):
            joint_name = joint.replace('.pos', '')
            if joint_name in current_pos:
                actual = current_pos[joint_name]
                error = abs(actual - expected)
                if error > tolerance:
                    errors.append(f"{joint_name}: expected {expected:.1f}°, got {actual:.1f}° (error: {error:.1f}°)")
    
    if errors:
        error_msg = "Robot position verification failed:\n  " + "\n  ".join(errors)
        logger.warning(error_msg)
        return (False, error_msg)
    
    return (True, "")


def IK_control_loop_improved(robot, keyboard, target_positions, xyzp_start_pos, control_freq=50):
    """
    Improved control loop with better error handling
    """
    current_xyz = {"x": xyzp_start_pos[0], "y": xyzp_start_pos[1], "z": xyzp_start_pos[2]}
    xyz_ranges = {
        "x": {"min": 0.01, "max": 0.45}, 
        "y": {"min": -0.45, "max": 0.45}, 
        "z": {"min": -0.4, "max": 0.45}
    }
    
    control_period = 1.0 / control_freq
    pitch = xyzp_start_pos[3]
    pitch_deg_per_sec = 50
    xyz_L1_m_per_sec = 0.2
    xyzp_initial = xyzp_start_pos
    
    # Statistics
    ik_failures = 0
    ik_successes = 0
    last_successful_xyz = current_xyz.copy()
    last_successful_pitch = pitch
    
    logger.info(f"Starting improved control loop at {control_freq}Hz")
    logger.info(f"Initial position: x={current_xyz['x']:.3f}, y={current_xyz['y']:.3f}, z={current_xyz['z']:.3f}, pitch={math.degrees(pitch):.1f}°")
    
    while True:
        try:
            t0 = time.perf_counter()
            
            keyboard_action = keyboard.get_action()
            
            if keyboard_action:
                changed_xyzp = False
                requested_xyz = current_xyz.copy()
                requested_pitch = pitch
                
                for key in keyboard_action.keys():
                    if key == 'x':
                        logger.info("Returning to start position")
                        requested_xyz['x'] = xyzp_initial[0]
                        requested_xyz['y'] = xyzp_initial[1]
                        requested_xyz['z'] = xyzp_initial[2]
                        requested_pitch = xyzp_initial[3]
                        changed_xyzp = True
                    
                    # Joint controls
                    joint_controls = {
                        'y': ('wrist_roll', -100 * control_period),
                        'h': ('wrist_roll', 100 * control_period),
                        'g': ('gripper', -100 * control_period),
                        't': ('gripper', 100 * control_period),
                    }
                    
                    # XYZ controls
                    xyz_controls = {
                        'w': ('x', xyz_L1_m_per_sec * control_period),
                        's': ('x', -xyz_L1_m_per_sec * control_period),
                        'd': ('y', xyz_L1_m_per_sec * control_period),
                        'a': ('y', -xyz_L1_m_per_sec * control_period),
                        'q': ('z', xyz_L1_m_per_sec * control_period),
                        'e': ('z', -xyz_L1_m_per_sec * control_period),
                    }
                    
                    # Pitch control
                    if key == 'r':
                        changed_xyzp = True
                        requested_pitch = min(requested_pitch + (pitch_deg_per_sec * control_period * math.pi / 180), math.pi)
                    elif key == 'f':
                        changed_xyzp = True
                        requested_pitch = max(requested_pitch - (pitch_deg_per_sec * control_period * math.pi / 180), -math.pi)
                    
                    if key in joint_controls:
                        joint_name, delta = joint_controls[key]
                        if joint_name in target_positions:
                            target_positions[joint_name] += delta
                    
                    elif key in xyz_controls:
                        changed_xyzp = True
                        coord, delta = xyz_controls[key]
                        requested_xyz[coord] = max(min(requested_xyz[coord] + delta, 
                                                       xyz_ranges[coord]["max"]), 
                                                   xyz_ranges[coord]["min"])
                
                # Try IK if position changed
                if changed_xyzp:
                    ik_result = xyzp_inverse_kinematics_improved(
                        robot, 
                        requested_xyz['x'], 
                        requested_xyz['y'], 
                        requested_xyz['z'], 
                        requested_pitch,
                        verbose=True
                    )
                    
                    if ik_result[0]:
                        # IK Success - update positions
                        target_positions['shoulder_pan'] = ik_result[1][0]
                        target_positions['shoulder_lift'] = ik_result[1][1]
                        target_positions['elbow_flex'] = ik_result[1][2]
                        target_positions['wrist_flex'] = ik_result[1][3]
                        
                        # Update current position
                        current_xyz = requested_xyz.copy()
                        pitch = requested_pitch
                        
                        # Track success
                        last_successful_xyz = current_xyz.copy()
                        last_successful_pitch = pitch
                        ik_successes += 1
                        
                    else:
                        # IK Failed - log and stay at last successful position
                        ik_failures += 1
                        logger.error(f"❌ IK FAILED (#{ik_failures}): {ik_result[2]}")
                        logger.error(f"   Requested: x={requested_xyz['x']:.3f}, y={requested_xyz['y']:.3f}, z={requested_xyz['z']:.3f}, pitch={math.degrees(requested_pitch):.1f}°")
                        logger.info(f"   Staying at: x={current_xyz['x']:.3f}, y={current_xyz['y']:.3f}, z={current_xyz['z']:.3f}, pitch={math.degrees(pitch):.1f}°")
                        
                        # Optionally: Try to recover by moving back slightly
                        # (commented out for now, but could be useful)
                        # if ik_failures > 3:
                        #     logger.warning("Multiple IK failures, returning to last known good position")
                        #     current_xyz = last_successful_xyz.copy()
                        #     pitch = last_successful_pitch
            
            # Create and send robot action
            robot_action = {}
            for joint_name, target_pos in target_positions.items():
                robot_action[f"{joint_name}.pos"] = target_pos
            
            if robot_action:
                logger.info(f"Position: x={current_xyz['x']:.3f}, y={current_xyz['y']:.3f}, z={current_xyz['z']:.3f}, pitch={math.degrees(pitch):.1f}° | IK: ✓{ik_successes} ✗{ik_failures}")
                robot.send_action(robot_action)
            
            # Wait for next cycle
            elapsed = time.perf_counter() - t0
            time.sleep(max(control_period - elapsed, 0.0))
            
        except KeyboardInterrupt:
            logger.info("User interrupted program")
            logger.info(f"Final statistics: IK successes={ik_successes}, failures={ik_failures}")
            break
        except Exception as e:
            logger.error(f"Control loop error: {e}")
            traceback.print_exc()
            break


# Keep other utility functions the same
def move_to_zero_position(robot, duration=3.0):
    """Move the robot to zero position"""
    zero_position = {
        'shoulder_pan': 0.0, 'shoulder_lift': 0.0, 'elbow_flex': 0.0, 
        'wrist_flex': 0.0, 'wrist_roll': 0.0, 'gripper': 0.0
    }
    move_to_position(robot, zero_position, duration)


def move_to_position(robot, position, duration=3.0):
    """Move to specified position"""
    from lerobot.utils.robot_utils import busy_wait
    
    current_positions = read_angles(robot)
    robot_action = {}
    for joint_name, target_pos in position.items():
        if joint_name in current_positions:
            robot_action[f"{joint_name}.pos"] = target_pos
    
    if robot_action:
        robot_action["x.vel"] = 0.0
        robot_action["y.vel"] = 0.0
        robot_action["theta.vel"] = 0.0
        logger.info(f"Commanding robot to move: {robot_action}")
        robot.send_action(robot_action)
    
    busy_wait(duration)


def read_and_print_angles(robot):
    """Print and return current joint angles"""
    start_positions = read_angles(robot)
    
    logger.info("Joint angles:")
    for joint_name, position in start_positions.items():
        logger.info(f"  {joint_name}: {int(position)}°")
    
    return start_positions


def main():
    from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
    from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
    from lerobot.utils.robot_utils import busy_wait
    
    try:
        from lerobot.utils.visualization_utils import init_rerun
    except (ImportError, AttributeError):
        def init_rerun(session_name: str = "so100_ik") -> None:
            logger.warning("Rerun visualization unavailable")
    
    # Configuration
    robot_config = SO100FollowerConfig(port="/dev/ttyACM0", id="marc", use_degrees=True)
    keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")
    
    robot = SO100Follower(robot_config)
    keyboard = KeyboardTeleop(keyboard_config)
    
    robot.connect()
    keyboard.connect()
    
    init_rerun(session_name="so100_ik")
    
    if not robot.is_connected or not keyboard.is_connected:
        raise ValueError("Robot or keyboard not connected!")
    
    # IK control
    try:
        # Safe starting configuration
        init_xyzp = (0.175, 0.0, 0.1, 0.0)
        init_angles = (0.0, -75.83, 44.43, 31.39)
        init_action = {
            'shoulder_pan.pos': init_angles[0], 
            'shoulder_lift.pos': init_angles[1], 
            'elbow_flex.pos': init_angles[2], 
            'wrist_flex.pos': init_angles[3], 
            'wrist_roll.pos': 0.0, 
            'gripper.pos': 0.0
        }
        init_action_no_pos = {
            'shoulder_pan': init_angles[0], 
            'shoulder_lift': init_angles[1], 
            'elbow_flex': init_angles[2], 
            'wrist_flex': init_angles[3], 
            'wrist_roll': 0.0, 
            'gripper': 0.0
        }
        
        logger.info("Moving to initial position...")
        robot.send_action(init_action)
        busy_wait(3)
        
        logger.info(f"Initial end effector position: x={init_xyzp[0]:.4f}, y={init_xyzp[1]:.4f}, z={init_xyzp[2]:.4f}, pitch={init_xyzp[3]:.4f}")
        
        logger.info("\n" + "="*50)
        logger.info("Keyboard control instructions:")
        logger.info("- W/S: X coordinate (forward/backward)")
        logger.info("- A/D: Y coordinate (left/right)")
        logger.info("- Q/E: Z coordinate (up/down)")
        logger.info("- R/F: Pitch adjustment")
        logger.info("- Y/H: Wrist roll")
        logger.info("- T/G: Gripper")
        logger.info("- X: Return to start")
        logger.info("="*50 + "\n")
        
        # Start improved control loop
        IK_control_loop_improved(robot, keyboard, init_action_no_pos, init_xyzp, control_freq=50)
        
    except Exception as e:
        logger.error(f"Program execution failed: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()
