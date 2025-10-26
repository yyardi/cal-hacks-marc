#!/usr/bin/env python

"""
Workspace Calibration Tool

This tool allows you to manually calibrate the paper center position
by jogging the robot to the desired location. The calibration is saved
to a file and automatically loaded on subsequent runs.
"""

import json
import os
import time
import sys

# Import IK function
sys.path.insert(0, '/mnt/project')
from teleoperate import xyzp_inverse_kinematics


CALIBRATION_FILE = "workspace_calibration.json"


def save_calibration(center_x, center_y, z_contact=None, page_width_mm=None, page_height_mm=None):
    """
    Save workspace calibration to a JSON file.
    
    Args:
        center_x: X coordinate of paper center (meters)
        center_y: Y coordinate of paper center (meters)
        z_contact: Optional z_contact height (meters)
        page_width_mm: Optional drawing width (mm) - for verification
        page_height_mm: Optional drawing height (mm) - for verification
    """
    calibration = {
        'center_x': center_x,
        'center_y': center_y,
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
    }
    
    if z_contact is not None:
        calibration['z_contact'] = z_contact
    
    if page_width_mm is not None and page_height_mm is not None:
        calibration['calibrated_width_mm'] = page_width_mm
        calibration['calibrated_height_mm'] = page_height_mm
    
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(calibration, f, indent=2)
    
    print(f"\n✓ Calibration saved to {CALIBRATION_FILE}")
    print(f"  Center: ({center_x:.3f}, {center_y:.3f})")
    if z_contact is not None:
        print(f"  Z-contact: {z_contact:.3f}")
    if page_width_mm is not None and page_height_mm is not None:
        print(f"  Dimensions: {page_width_mm:.1f}mm × {page_height_mm:.1f}mm")


def load_calibration():
    """
    Load workspace calibration from JSON file.
    
    Returns:
        dict: Calibration data or None if file doesn't exist
    """
    if not os.path.exists(CALIBRATION_FILE):
        return None
    
    try:
        with open(CALIBRATION_FILE, 'r') as f:
            calibration = json.load(f)
        
        print(f"\n✓ Loaded calibration from {CALIBRATION_FILE}")
        print(f"  Center: ({calibration['center_x']:.3f}, {calibration['center_y']:.3f})")
        if 'z_contact' in calibration:
            print(f"  Z-contact: {calibration['z_contact']:.3f}")
        if 'calibrated_width_mm' in calibration and 'calibrated_height_mm' in calibration:
            print(f"  Calibrated for: {calibration['calibrated_width_mm']:.1f}mm × {calibration['calibrated_height_mm']:.1f}mm")
        print(f"  Saved: {calibration.get('timestamp', 'unknown')}")
        
        return calibration
    
    except Exception as e:
        print(f"⚠️  Warning: Could not load calibration: {e}")
        return None


def interactive_center_calibration(controller, initial_x=0.21, initial_y=0.0, initial_z=0.0, initial_pitch=0.0):
    """
    Interactive calibration to teach the robot the paper center position.
    
    Args:
        controller: RobotMovementController instance
        initial_x: Starting X position (meters)
        initial_y: Starting Y position (meters)
        initial_z: Starting Z position (meters)
        initial_pitch: Starting pitch (radians)
    
    Returns:
        tuple: (center_x, center_y, z_contact) or (None, None, None) if cancelled
    """
    print("\n" + "="*70)
    print("🎯 INTERACTIVE WORKSPACE CENTER CALIBRATION")
    print("="*70)
    
    print("\nThis tool lets you manually position the robot at the CENTER of your paper.")
    print("The robot will remember this position and center all future drawings there.")
    print("\nYou'll be able to move the robot in X, Y, and Z using keyboard controls.")
    
    response = input("\nStart center calibration? (yes/no): ")
    if response.lower() not in ['yes', 'y']:
        print("Skipping calibration")
        return None, None, None
    
    # Current position
    current_x = initial_x
    current_y = initial_y
    current_z = initial_z
    current_pitch = initial_pitch
    
    # Movement increments
    xy_increment = 0.005  # 5mm steps
    z_increment = 0.001   # 1mm steps
    
    print("\n" + "="*70)
    print("MOVING TO INITIAL POSITION")
    print("="*70)
    print(f"Initial position: x={current_x:.3f}, y={current_y:.3f}, z={current_z:.3f}")
    
    # Move to initial position
    success, joint_angles = xyzp_inverse_kinematics(
        controller.robot, current_x, current_y, current_z, current_pitch
    )
    
    if not success:
        print("❌ Cannot reach initial position with IK!")
        return None, None, None
    
    action = {
        'shoulder_pan.pos': joint_angles[0],
        'shoulder_lift.pos': joint_angles[1],
        'elbow_flex.pos': joint_angles[2],
        'wrist_flex.pos': joint_angles[3]
    }
    controller.robot.send_action(action)
    time.sleep(2.0)
    
    print(f"\n✓ Robot at initial position")
    
    # Interactive jogging loop
    print("\n" + "="*70)
    print("JOG ROBOT TO PAPER CENTER")
    print("="*70)
    print("\nControls:")
    print("  'i' = Forward (+X, 5mm)")
    print("  'k' = Backward (-X, 5mm)")
    print("  'j' = Left (-Y, 5mm)")
    print("  'l' = Right (+Y, 5mm)")
    print("  'u' = Up (+Z, 1mm)")
    print("  'm' = Down (-Z, 1mm)")
    print("  'I' = Forward (+X, 20mm)  [FAST]")
    print("  'K' = Backward (-X, 20mm) [FAST]")
    print("  'J' = Left (-Y, 20mm)     [FAST]")
    print("  'L' = Right (+Y, 20mm)    [FAST]")
    print("  'U' = Up (+Z, 5mm)        [FAST]")
    print("  'M' = Down (-Z, 5mm)      [FAST]")
    print("  'q' = Save this position as center")
    print("  'x' = Cancel calibration")
    
    print(f"\n📍 Current position: x={current_x:.3f}, y={current_y:.3f}, z={current_z:.3f}")
    print("\n⚠️  Jog the marker to the CENTER of your paper, then press 'q'")
    
    while True:
        command = input("\nCommand (i/k/j/l/u/m/q/x): ").strip()
        
        if command == 'q':
            # Save this position
            print(f"\n✓ Position saved as paper center!")
            print(f"  X = {current_x:.3f} m ({current_x*1000:.1f} mm)")
            print(f"  Y = {current_y:.3f} m ({current_y*1000:.1f} mm)")
            print(f"  Z = {current_z:.3f} m ({current_z*1000:.1f} mm)")
            return current_x, current_y, current_z
        
        elif command == 'x':
            print("\n❌ Calibration cancelled")
            return None, None, None
        
        # Parse movement commands
        moves = {
            'i': ('x', xy_increment),
            'k': ('x', -xy_increment),
            'j': ('y', -xy_increment),
            'l': ('y', xy_increment),
            'u': ('z', z_increment),
            'm': ('z', -z_increment),
            'I': ('x', xy_increment * 4),
            'K': ('x', -xy_increment * 4),
            'J': ('y', -xy_increment * 4),
            'L': ('y', xy_increment * 4),
            'U': ('z', z_increment * 5),
            'M': ('z', -z_increment * 5),
        }
        
        if command in moves:
            axis, delta = moves[command]
            
            # Update position
            if axis == 'x':
                current_x += delta
            elif axis == 'y':
                current_y += delta
            elif axis == 'z':
                current_z += delta
            
            # Try to move to new position
            success, joint_angles = xyzp_inverse_kinematics(
                controller.robot, current_x, current_y, current_z, current_pitch
            )
            
            if not success:
                print("⚠️  Warning: Cannot reach this position with IK!")
                print("Reverting to previous position")
                # Revert
                if axis == 'x':
                    current_x -= delta
                elif axis == 'y':
                    current_y -= delta
                elif axis == 'z':
                    current_z -= delta
            else:
                # Move robot
                action = {
                    'shoulder_pan.pos': joint_angles[0],
                    'shoulder_lift.pos': joint_angles[1],
                    'elbow_flex.pos': joint_angles[2],
                    'wrist_flex.pos': joint_angles[3]
                }
                controller.robot.send_action(action)
                time.sleep(0.3)  # Brief pause for movement
                
                print(f"Moved {axis} by {delta*1000:.1f}mm")
                print(f"📍 Position: x={current_x:.3f}, y={current_y:.3f}, z={current_z:.3f}")
        
        else:
            print("Invalid command. Use i/k/j/l/u/m/I/K/J/L/U/M/q/x")


def calculate_origin_from_center(center_x, center_y, page_width_mm, page_height_mm):
    """
    Calculate the drawing origin (bottom-left) from the calibrated center position.
    
    Args:
        center_x: X coordinate of paper center (meters)
        center_y: Y coordinate of paper center (meters)
        page_width_mm: Drawing width (mm)
        page_height_mm: Drawing height (mm)
    
    Returns:
        tuple: (origin_x, origin_y) in meters
    """
    page_width_m = page_width_mm * 0.001
    page_height_m = page_height_mm * 0.001
    
    # Origin is center minus half the drawing size
    origin_x = center_x - (page_width_m / 2)
    origin_y = center_y - (page_height_m / 2)
    
    return origin_x, origin_y


def calibrate_workspace_bounds_from_center(center_x, center_y, margin_mm=10):
    """
    Calculate workspace bounds from calibrated center.
    Assumes roughly 80mm x 80mm reachable area centered on the calibrated point.
    
    Args:
        center_x: Calibrated center X (meters)
        center_y: Calibrated center Y (meters)
        margin_mm: Safety margin to keep away from edges (mm)
    
    Returns:
        dict: workspace boundaries
    """
    # Assume reachable area is roughly 80mm x 80mm centered on calibrated point
    margin_m = margin_mm * 0.001
    half_width = 0.040 - margin_m   # 40mm - margin
    half_height = 0.040 - margin_m  # 40mm - margin
    
    return {
        'x_min': center_x - half_width,
        'x_max': center_x + half_width,
        'y_min': center_y - half_height,
        'y_max': center_y + half_height
    }


if __name__ == "__main__":
    print("\n" + "="*70)
    print("WORKSPACE CALIBRATION UTILITY")
    print("="*70)
    print("\nThis utility helps you calibrate the paper center position.")
    print("Run this before drawing to ensure proper centering.")
    print("\nTo use in your drawing script, import these functions:")
    print("  from workspace_calibration import load_calibration, calculate_origin_from_center")
