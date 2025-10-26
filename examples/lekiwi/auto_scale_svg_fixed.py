#!/usr/bin/env python

"""
SVG Auto-Scaler with Workspace Calibration (FIXED DIMENSIONS)

IMPORTANT: This version uses the SAME dimensions for calibration and drawing.
The dimensions you calibrate with must match the dimensions you draw with!
"""

import math
import time
import sys
from xml.dom import minidom
from svgpathtools import parse_path
from robot_movement import RobotMovementController, DrawingPath, SO100Follower, SO100FollowerConfig

# Import calibration functions
from workspace_calibration import (
    load_calibration, 
    save_calibration,
    interactive_center_calibration,
    calculate_origin_from_center
)


def get_svg_viewbox(svg_file):
    """Extract viewBox or dimensions from SVG file."""
    doc = minidom.parse(svg_file)
    svg_elem = doc.getElementsByTagName('svg')[0]
    
    viewbox = svg_elem.getAttribute('viewBox')
    if viewbox:
        parts = viewbox.split()
        if len(parts) == 4:
            min_x, min_y, width, height = map(float, parts)
            return width, height, min_x, min_y
    
    width = svg_elem.getAttribute('width')
    height = svg_elem.getAttribute('height')
    
    if width and height:
        width = float(width.replace('px', '').replace('pt', '').replace('mm', ''))
        height = float(height.replace('px', '').replace('pt', '').replace('mm', ''))
        return width, height, 0, 0
    
    return None, None, 0, 0


def get_path_bounds(svg_file):
    """Calculate bounding box of all paths in SVG."""
    doc = minidom.parse(svg_file)
    
    all_x = []
    all_y = []
    
    for path_elem in doc.getElementsByTagName('path'):
        d = path_elem.getAttribute('d')
        if not d:
            continue
        
        try:
            svg_path = parse_path(d)
            for seg in svg_path:
                all_x.append(seg.start.real)
                all_y.append(seg.start.imag)
                all_x.append(seg.end.real)
                all_y.append(seg.end.imag)
        except Exception:
            continue
    
    if not all_x:
        return None, None, None, None
    
    return min(all_x), min(all_y), max(all_x), max(all_y)


def analyze_svg(svg_file):
    """Analyze SVG and show info."""
    print("\n" + "="*70)
    print("ANALYZING SVG")
    print("="*70)
    
    svg_width, svg_height, svg_min_x, svg_min_y = get_svg_viewbox(svg_file)
    min_x, min_y, max_x, max_y = get_path_bounds(svg_file)
    
    if min_x is None:
        print("❌ No paths found in SVG")
        return None
    
    actual_width = max_x - min_x
    actual_height = max_y - min_y
    aspect_ratio = actual_width / actual_height if actual_height > 0 else 1.0
    
    print(f"\nSVG info:")
    print(f"  ViewBox: {svg_width} × {svg_height} px")
    print(f"  Actual content: {actual_width:.1f} × {actual_height:.1f} px")
    print(f"  Aspect ratio: {aspect_ratio:.2f}")
    
    return {
        'svg_size': (svg_width, svg_height),
        'actual_bounds': (min_x, min_y, max_x, max_y),
        'actual_size': (actual_width, actual_height),
        'aspect_ratio': aspect_ratio
    }


def load_and_scale_svg(svg_file, actual_bounds, page_width_mm, page_height_mm,
                       robot_origin_x, robot_origin_y):
    """Load SVG and transform to robot coordinates."""
    min_x, min_y, max_x, max_y = actual_bounds
    svg_width = max_x - min_x
    svg_height = max_y - min_y
    
    doc = minidom.parse(svg_file)
    all_paths = []
    
    for path_elem in doc.getElementsByTagName('path'):
        d = path_elem.getAttribute('d')
        if not d:
            continue
        
        try:
            svg_path = parse_path(d)
            points = []
            
            num_samples = max(10, len(svg_path) * 5)
            for i in range(num_samples + 1):
                t = i / num_samples
                point = svg_path.point(t)
                x_svg = point.real
                y_svg = point.imag
                
                # Normalize
                x_normalized = x_svg - min_x
                y_normalized = y_svg - min_y
                
                # SVG pixels → Page mm
                x_mm = (x_normalized / svg_width) * page_width_mm
                y_mm = (y_normalized / svg_height) * page_height_mm
                
                # Page mm → Robot meters
                x_robot = robot_origin_x + (x_mm * 0.001)
                y_robot = robot_origin_y + (y_mm * 0.001)
                
                points.append((x_robot, y_robot))
            
            if points:
                all_paths.append(points)
        except Exception:
            pass
    
    return all_paths


def test_ik_reachability(controller, x, y, z, pitch):
    """Test if a position is reachable with IK."""
    sys.path.insert(0, '/mnt/project')
    from teleoperate import xyzp_inverse_kinematics
    
    success, _ = xyzp_inverse_kinematics(controller.robot, x, y, z, pitch)
    return success, success


def validate_reachability(robot_paths, controller, z_contact, pitch=0.0, 
                         sample_rate=10, min_threshold=0.90):
    """Validate that most drawing points are reachable."""
    print("\n" + "="*70)
    print("VALIDATING REACHABILITY")
    print("="*70)
    
    total_points = 0
    reachable_points = 0
    reachable_map = []
    
    for path in robot_paths:
        path_reachable = []
        for i, (x, y) in enumerate(path):
            if i % sample_rate == 0:  # Sample every Nth point
                total_points += 1
                reachable, _ = test_ik_reachability(controller, x, y, z_contact, pitch)
                if reachable:
                    reachable_points += 1
                path_reachable.append(reachable)
        reachable_map.append(path_reachable)
    
    fraction = reachable_points / total_points if total_points > 0 else 0
    
    print(f"\nReachability test:")
    print(f"  Sampled points: {total_points}")
    print(f"  Reachable: {reachable_points} ({fraction*100:.1f}%)")
    print(f"  Unreachable: {total_points - reachable_points} ({(1-fraction)*100:.1f}%)")
    
    if fraction >= min_threshold:
        print(f"  ✓ Coverage acceptable (≥{min_threshold*100:.0f}%)")
    else:
        print(f"  ⚠ Coverage below threshold ({min_threshold*100:.0f}%)")
    
    return fraction, reachable_map


def interactive_z_calibration(controller, first_x, first_y, initial_z_contact):
    """Interactive z-height calibration."""
    print("\n" + "="*70)
    print("⚙️  INTERACTIVE Z-HEIGHT CALIBRATION")
    print("="*70)
    
    print("\nThis allows you to fine-tune the marker contact height.")
    print("The robot will move to the first drawing point.")
    
    response = input("\nStart z-height calibration? (yes/no): ")
    if response.lower() not in ['yes', 'y']:
        print("Skipping calibration, using z_contact =", initial_z_contact)
        return initial_z_contact
    
    current_z = initial_z_contact
    z_increment = 0.001
    
    print(f"\nFirst drawing point: x={first_x:.3f}, y={first_y:.3f}")
    
    sys.path.insert(0, '/mnt/project')
    from teleoperate import xyzp_inverse_kinematics
    
    # Move to first point
    success, joint_angles = xyzp_inverse_kinematics(
        controller.robot, first_x, first_y, current_z, 0.0
    )
    
    if not success:
        print("❌ Cannot reach first waypoint!")
        return initial_z_contact
    
    action = {
        'shoulder_pan.pos': joint_angles[0],
        'shoulder_lift.pos': joint_angles[1],
        'elbow_flex.pos': joint_angles[2],
        'wrist_flex.pos': joint_angles[3]
    }
    controller.robot.send_action(action)
    time.sleep(2.0)
    
    print(f"\n✓ Robot at first waypoint")
    print("\nControls:")
    print("  'w' = UP 1mm   's' = DOWN 1mm")
    print("  'W' = UP 5mm   'S' = DOWN 5mm")
    print("  'q' = Save this height")
    
    while True:
        print(f"\nCurrent z = {current_z:.3f} m ({current_z*1000:.1f} mm)")
        command = input("Command (w/s/W/S/q): ").strip()
        
        if command == 'q':
            print(f"\n✓ Z-height calibration complete: {current_z:.3f} m")
            return current_z
        
        elif command == 'w':
            current_z += z_increment
        elif command == 's':
            current_z -= z_increment
        elif command == 'W':
            current_z += z_increment * 5
        elif command == 'S':
            current_z -= z_increment * 5
        else:
            print("Invalid command")
            continue
        
        # Move to new z
        success, joint_angles = xyzp_inverse_kinematics(
            controller.robot, first_x, first_y, current_z, 0.0
        )
        
        if success:
            action = {
                'shoulder_pan.pos': joint_angles[0],
                'shoulder_lift.pos': joint_angles[1],
                'elbow_flex.pos': joint_angles[2],
                'wrist_flex.pos': joint_angles[3]
            }
            controller.robot.send_action(action)
            time.sleep(0.3)
        else:
            print("⚠️  Cannot reach this z!")
            if command in ['w', 's']:
                current_z -= z_increment if command == 'w' else -z_increment
            else:
                current_z -= z_increment * 5 if command == 'W' else -z_increment * 5


def draw_scaled_svg_calibrated(svg_file, controller, analysis, 
                               page_width_mm, page_height_mm,
                               z_contact, z_safe, 
                               pitch=0.0, min_threshold=0.90,
                               calibrated_center_x=None, calibrated_center_y=None):
    """
    Draw SVG using calibrated center position and EXPLICIT dimensions.
    
    Args:
        page_width_mm: Drawing width in mm (MUST match calibration!)
        page_height_mm: Drawing height in mm (MUST match calibration!)
        calibrated_center_x: Calibrated X coordinate of paper center (meters)
        calibrated_center_y: Calibrated Y coordinate of paper center (meters)
    """
    print("\n" + "="*70)
    print("PREPARING TO DRAW")
    print("="*70)
    
    actual_bounds = analysis['actual_bounds']
    
    print(f"\n⚠️  CRITICAL: Drawing dimensions")
    print(f"  Width: {page_width_mm:.1f} mm")
    print(f"  Height: {page_height_mm:.1f} mm")
    print(f"  These MUST match what you used in find_optimal_center.py!")
    
    # Calculate origin from calibrated center
    if calibrated_center_x is not None and calibrated_center_y is not None:
        robot_origin_x, robot_origin_y = calculate_origin_from_center(
            calibrated_center_x, calibrated_center_y,
            page_width_mm, page_height_mm
        )
        print(f"\nUsing CALIBRATED center position:")
        print(f"  Center: ({calibrated_center_x:.3f}, {calibrated_center_y:.3f})")
        print(f"  Origin: ({robot_origin_x:.3f}, {robot_origin_y:.3f})")
        print(f"  Drawing extent:")
        print(f"    X: [{robot_origin_x:.3f}, {robot_origin_x + page_width_mm*0.001:.3f}]")
        print(f"    Y: [{robot_origin_y:.3f}, {robot_origin_y + page_height_mm*0.001:.3f}]")
    else:
        # Fallback to default centering
        print(f"\n⚠️  No calibration found, using default centering")
        workspace_x_min, workspace_x_max = 0.17, 0.25
        workspace_y_min, workspace_y_max = -0.09, 0.09
        
        page_width_m = page_width_mm * 0.001
        page_height_m = page_height_mm * 0.001
        
        workspace_center_x = (workspace_x_min + workspace_x_max) / 2
        workspace_center_y = (workspace_y_min + workspace_y_max) / 2
        
        robot_origin_x = workspace_center_x - page_width_m / 2
        robot_origin_y = workspace_center_y - page_height_m / 2
        
        print(f"  Origin: ({robot_origin_x:.3f}, {robot_origin_y:.3f})")
    
    # Load and scale SVG
    robot_paths = load_and_scale_svg(
        svg_file, actual_bounds, page_width_mm, page_height_mm,
        robot_origin_x, robot_origin_y
    )
    
    print(f"\nLoaded {len(robot_paths)} paths")
    
    if len(robot_paths) == 0 or len(robot_paths[0]) == 0:
        print("❌ No valid paths to draw!")
        return False
    
    # Get first waypoint for z-calibration
    first_x, first_y = robot_paths[0][0]
    
    # Z-calibration
    calibrated_z_contact = interactive_z_calibration(
        controller, first_x, first_y, z_contact
    )
    z_contact = calibrated_z_contact
    z_safe = z_contact + 0.05
    
    # Validate reachability
    reachable_fraction, _ = validate_reachability(
        robot_paths, controller, z_contact, pitch=pitch,
        sample_rate=10, min_threshold=min_threshold
    )
    
    if reachable_fraction < min_threshold:
        print(f"\n⚠ Coverage too low ({reachable_fraction*100:.1f}%)")
        print(f"\n🔍 Diagnosis:")
        print(f"  - Did you run find_optimal_center.py with {page_width_mm:.0f}x{page_height_mm:.0f}mm?")
        print(f"  - If not, dimensions don't match!")
        print(f"  - Try reducing size or re-running optimal finder")
        response = input("\nContinue anyway? (y/n): ")
        if response.lower() != 'y':
            return False
    
    # Confirmation
    print("\n" + "="*70)
    print("⚠️  READY TO START DRAWING")
    print("="*70)
    print(f"\nParameters:")
    print(f"  Dimensions: {page_width_mm:.1f}mm × {page_height_mm:.1f}mm")
    print(f"  z_contact: {z_contact:.3f} m")
    print(f"  Coverage: {reachable_fraction*100:.1f}%")
    print(f"  Origin: ({robot_origin_x:.3f}, {robot_origin_y:.3f})")
    
    response = input("\nStart drawing? (yes/no): ")
    if response.lower() not in ['yes', 'y']:
        print("❌ Cancelled")
        return False
    
    # Draw
    print(f"\nDrawing {len(robot_paths)} paths...")
    
    for path_idx, robot_path in enumerate(robot_paths):
        print(f"\n  Path {path_idx+1}/{len(robot_paths)}")
        
        segments = []
        current_segment = []
        
        for x, y in robot_path:
            reachable, _ = test_ik_reachability(controller, x, y, z_contact, pitch)
            
            if reachable:
                current_segment.append((x, y))
            else:
                if len(current_segment) >= 2:
                    segments.append(current_segment)
                current_segment = []
        
        if len(current_segment) >= 2:
            segments.append(current_segment)
        
        # Draw segments
        for segment in segments:
            path = DrawingPath(z_contact=z_contact, z_safe=z_safe)
            path.add_point(segment[0][0], segment[0][1], pen_down=False)
            
            for x, y in segment:
                path.add_point(x, y, pen_down=True)
            
            path.add_point(segment[-1][0], segment[-1][1], pen_down=False)
            path.execute(controller, move_duration=0.3)
    
    print(f"\n✓ Drawing complete!")
    return True


def main():
    if len(sys.argv) < 2:
        print("Usage: python auto_scale_svg_fixed.py <svg_file> [width_mm] [height_mm]")
        print("\nExamples:")
        print("  python auto_scale_svg_fixed.py butterfly.svg")
        print("  python auto_scale_svg_fixed.py butterfly.svg 70 70")
        sys.exit(1)
    
    svg_file = sys.argv[1]
    
    # Analyze SVG
    analysis = analyze_svg(svg_file)
    if not analysis:
        print("\n✗ Failed to analyze SVG")
        sys.exit(1)
    
    # Get drawing dimensions (CRITICAL!)
    print("\n" + "="*70)
    print("⚠️  DRAWING DIMENSIONS (IMPORTANT!)")
    print("="*70)
    print("\nThese dimensions MUST match what you used in find_optimal_center.py!")
    print("If you don't remember, check the output from find_optimal_center.py")
    
    if len(sys.argv) >= 4:
        # Dimensions provided as command line args
        page_width_mm = float(sys.argv[2])
        page_height_mm = float(sys.argv[3])
        print(f"\nUsing command-line dimensions: {page_width_mm:.1f}mm × {page_height_mm:.1f}mm")
    else:
        # Ask user
        aspect_ratio = analysis['aspect_ratio']
        print(f"\nSVG aspect ratio: {aspect_ratio:.2f}")
        
        width_input = input("\nEnter drawing width in mm (or Enter for 70): ").strip()
        page_width_mm = float(width_input) if width_input else 70.0
        
        height_input = input(f"Enter drawing height in mm (or Enter for {page_width_mm/aspect_ratio:.1f}): ").strip()
        if height_input:
            page_height_mm = float(height_input)
        else:
            page_height_mm = page_width_mm / aspect_ratio
    
    print(f"\n📐 Drawing will be: {page_width_mm:.1f}mm × {page_height_mm:.1f}mm")
    
    # Initialize robot
    print("\n" + "="*70)
    print("INITIALIZING ROBOT")
    print("="*70)
    robot_config = SO100FollowerConfig(
        port="/dev/ttyACM0",
        id="marc",
        use_degrees=True
    )
    robot = SO100Follower(robot_config)
    robot.connect()
    
    if not robot.is_connected:
        raise ValueError("Robot not connected!")
    
    controller = RobotMovementController(robot)
    
    # Load calibration
    print("\n" + "="*70)
    print("WORKSPACE CALIBRATION")
    print("="*70)
    
    calibration = load_calibration()
    
    if calibration:
        print("\n✓ Loaded saved calibration")
        print(f"  Center: ({calibration['center_x']:.3f}, {calibration['center_y']:.3f})")
        print(f"\n⚠️  Make sure this was calibrated with {page_width_mm:.0f}x{page_height_mm:.0f}mm!")
        response = input("Use this calibration? (yes/no): ")
        if response.lower() not in ['yes', 'y']:
            print("\n❌ Please run find_optimal_center.py first with correct dimensions:")
            print(f"   python find_optimal_center.py {svg_file} {page_width_mm:.0f} {page_height_mm:.0f}")
            sys.exit(1)
    else:
        print("\n❌ No calibration found!")
        print("\nYou need to run find_optimal_center.py first:")
        print(f"  python find_optimal_center.py {svg_file} {page_width_mm:.0f} {page_height_mm:.0f}")
        sys.exit(1)
    
    # Get z_contact
    z_contact = calibration.get('z_contact', 0.0)
    z_contact_input = input(f"\nZ-contact (Enter for {z_contact:.3f}): ").strip()
    if z_contact_input:
        z_contact = float(z_contact_input)
    
    z_safe = z_contact + 0.05
    
    # Get pitch
    pitch_input = input("\nPitch in radians (Enter for 0.0): ").strip()
    pitch = float(pitch_input) if pitch_input else 0.0
    
    # Get threshold
    threshold_input = input("\nMin coverage threshold (Enter for 0.90): ").strip()
    min_threshold = float(threshold_input) if threshold_input else 0.90
    
    # Draw
    print("\n" + "="*70)
    print("STARTING DRAW SEQUENCE")
    print("="*70)
    
    try:
        success = draw_scaled_svg_calibrated(
            svg_file,
            controller,
            analysis,
            page_width_mm,
            page_height_mm,
            z_contact,
            z_safe,
            pitch=pitch,
            min_threshold=min_threshold,
            calibrated_center_x=calibration['center_x'],
            calibrated_center_y=calibration['center_y']
        )
        
        if success:
            print("\n✓ Drawing completed!")
        else:
            print("\n✗ Drawing failed")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  INTERRUPTED")
        controller.move_to_safe_position(duration=3.0)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        controller.move_to_safe_position(duration=3.0)
    finally:
        print("\nReturning to safe position...")
        controller.move_to_safe_position(duration=3.0)
        print("\n✓ Complete")


if __name__ == "__main__":
    main()
