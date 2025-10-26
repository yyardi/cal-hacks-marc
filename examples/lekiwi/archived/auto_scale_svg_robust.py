#!/usr/bin/env python

"""
SVG Auto-Scaler for Robot Workspace (ROBUST VERSION)

This version validates IK reachability and gracefully handles unreachable points.
"""

import math
from xml.dom import minidom
from svgpathtools import parse_path
from robot_movement import RobotMovementController, DrawingPath, SO100Follower, SO100FollowerConfig


def get_svg_viewbox(svg_file):
    """
    Extract viewBox or dimensions from SVG file.
    
    Returns:
        (width, height, min_x, min_y)
    """
    doc = minidom.parse(svg_file)
    svg_elem = doc.getElementsByTagName('svg')[0]
    
    # Try viewBox first
    viewbox = svg_elem.getAttribute('viewBox')
    if viewbox:
        parts = viewbox.split()
        if len(parts) == 4:
            min_x, min_y, width, height = map(float, parts)
            return width, height, min_x, min_y
    
    # Try width/height attributes
    width = svg_elem.getAttribute('width')
    height = svg_elem.getAttribute('height')
    
    if width and height:
        # Remove units
        width = float(width.replace('px', '').replace('pt', '').replace('mm', ''))
        height = float(height.replace('px', '').replace('pt', '').replace('mm', ''))
        return width, height, 0, 0
    
    return None, None, 0, 0


def get_path_bounds(svg_file):
    """
    Calculate bounding box of all paths in SVG.
    
    Returns:
        (min_x, min_y, max_x, max_y)
    """
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


def test_ik_reachability(controller, x, y, z, pitch=0.0):
    """
    Test if a position is reachable by the robot using IK.
    
    Returns:
        (reachable: bool, joint_angles: tuple or None)
    """
    try:
        # Import IK function from teleoperate
        import sys
        import os
        # Add the project directory to path
        sys.path.insert(0, '/mnt/project')
        from teleoperate import xyzp_inverse_kinematics
        
        success, joint_angles = xyzp_inverse_kinematics(
            controller.robot, 
            x, 
            y, 
            z, 
            pitch
        )
        return success, joint_angles if success else None
    except Exception as e:
        # If IK fails, assume unreachable
        return False, None


def analyze_svg(svg_file):
    """
    Analyze SVG and provide scaling recommendations.
    """
    print("="*70)
    print(f"ANALYZING: {svg_file}")
    print("="*70)
    
    # Get viewBox
    vb_width, vb_height, vb_min_x, vb_min_y = get_svg_viewbox(svg_file)
    
    if vb_width and vb_height:
        print(f"\nSVG ViewBox:")
        print(f"  Origin: ({vb_min_x:.1f}, {vb_min_y:.1f})")
        print(f"  Size: {vb_width:.1f} × {vb_height:.1f} px")
    else:
        print(f"\nSVG ViewBox: Not found or invalid")
    
    # Get actual bounds of paths
    min_x, min_y, max_x, max_y = get_path_bounds(svg_file)
    
    if min_x is None:
        print(f"\n✗ Could not find any paths in SVG!")
        return None
    
    actual_width = max_x - min_x
    actual_height = max_y - min_y
    
    print(f"\nActual Path Bounds:")
    print(f"  X range: {min_x:.1f} to {max_x:.1f}")
    print(f"  Y range: {min_y:.1f} to {max_y:.1f}")
    print(f"  Size: {actual_width:.1f} × {actual_height:.1f} px")
    
    # Check for viewBox mismatch
    if vb_width and vb_height:
        if abs(actual_width - vb_width) > 10 or abs(actual_height - vb_height) > 10:
            print(f"\n⚠ WARNING: ViewBox and actual paths don't match!")
            print(f"  ViewBox says: {vb_width:.0f} × {vb_height:.0f}")
            print(f"  Paths actually span: {actual_width:.0f} × {actual_height:.0f}")
            print(f"  → Using actual path bounds for scaling")
    
    # Use actual bounds for scaling
    svg_width = actual_width
    svg_height = actual_height
    
    # Robot workspace
    print(f"\nRobot Workspace:")
    print(f"  X: 0.15 to 0.25 m (100mm range)")
    print(f"  Y: -0.10 to 0.10 m (200mm range)")
    print(f"  Safe drawing area: ~80mm × 180mm (with margins)")
    
    # Calculate scaling options
    aspect_ratio = svg_width / svg_height if svg_height > 0 else 1.0
    
    print(f"\nSVG Aspect Ratio: {aspect_ratio:.2f}")
    
    # Option 1: Fit width to robot X (80mm with margin)
    target_width_mm_1 = 80  # Conservative
    scale_1 = target_width_mm_1 / svg_width if svg_width > 0 else 1
    scaled_height_1 = svg_height * scale_1
    
    # Option 2: Fit height to robot Y (150mm with margin)  
    target_height_mm_2 = 150
    scale_2 = target_height_mm_2 / svg_height if svg_height > 0 else 1
    scaled_width_2 = svg_width * scale_2
    
    # Option 3: Fit both (use smaller scale factor)
    scale_x = 80 / svg_width if svg_width > 0 else 1
    scale_y = 150 / svg_height if svg_height > 0 else 1
    scale_3 = min(scale_x, scale_y)
    scaled_width_3 = svg_width * scale_3
    scaled_height_3 = svg_height * scale_3
    
    print(f"\n" + "="*70)
    print("SCALING OPTIONS")
    print("="*70)
    
    print(f"\nOption 1: Fit to X-axis (width)")
    print(f"  page_width_mm = {target_width_mm_1}")
    print(f"  page_height_mm = {scaled_height_1:.1f}")
    print(f"  Fits: {'✓ Yes' if scaled_height_1 <= 150 else '✗ Too tall'}")
    
    print(f"\nOption 2: Fit to Y-axis (height)")
    print(f"  page_width_mm = {scaled_width_2:.1f}")
    print(f"  page_height_mm = {target_height_mm_2}")
    print(f"  Fits: {'✓ Yes' if scaled_width_2 <= 80 else '✗ Too wide'}")
    
    print(f"\nOption 3: Best fit (recommended)")
    print(f"  page_width_mm = {scaled_width_3:.1f}")
    print(f"  page_height_mm = {scaled_height_3:.1f}")
    print(f"  Fits: ✓ Yes (maximizes size while fitting)")
    
    return {
        'svg_width': svg_width,
        'svg_height': svg_height,
        'viewbox_width': vb_width,
        'viewbox_height': vb_height,
        'actual_bounds': (min_x, min_y, max_x, max_y),
        'option1': (target_width_mm_1, scaled_height_1),
        'option2': (scaled_width_2, target_height_mm_2),
        'option3': (scaled_width_3, scaled_height_3),
        'recommended': (scaled_width_3, scaled_height_3)
    }


def load_and_scale_svg(svg_file, actual_bounds, page_width_mm, page_height_mm,
                       robot_origin_x=0.15, robot_origin_y=-0.09):
    """
    Load SVG and transform to robot coordinates with proper scaling.
    Uses ACTUAL path bounds instead of viewBox to handle coordinate mismatches.
    
    Args:
        svg_file: Path to SVG file
        actual_bounds: (min_x, min_y, max_x, max_y) from get_path_bounds()
        page_width_mm: Physical width to scale to in mm
        page_height_mm: Physical height to scale to in mm
        robot_origin_x: Robot X coordinate of page origin (bottom-left)
        robot_origin_y: Robot Y coordinate of page origin (bottom-left)
    
    Returns:
        List of paths with robot coordinates
    """
    from xml.dom import minidom
    from svgpathtools import parse_path
    
    min_x, min_y, max_x, max_y = actual_bounds
    svg_width = max_x - min_x
    svg_height = max_y - min_y
    
    doc = minidom.parse(svg_file)
    all_paths = []
    
    print(f"\nTransformation (using actual path bounds):")
    print(f"  SVG bounds: ({min_x:.0f}, {min_y:.0f}) to ({max_x:.0f}, {max_y:.0f})")
    print(f"  SVG size: {svg_width:.0f} × {svg_height:.0f} px")
    print(f"  Page size: {page_width_mm:.1f} × {page_height_mm:.1f} mm")
    print(f"  Robot origin: ({robot_origin_x:.3f}, {robot_origin_y:.3f}) m")
    
    for path_elem in doc.getElementsByTagName('path'):
        d = path_elem.getAttribute('d')
        if not d:
            continue
        
        try:
            svg_path = parse_path(d)
            points = []
            
            # Sample points along path
            num_samples = max(10, len(svg_path) * 5)
            for i in range(num_samples + 1):
                t = i / num_samples
                point = svg_path.point(t)
                x_svg = point.real
                y_svg = point.imag
                
                # STEP 1: Normalize to [0, svg_width] and [0, svg_height]
                x_normalized = x_svg - min_x
                y_normalized = y_svg - min_y
                
                # STEP 2: SVG pixels → Page mm
                x_mm = (x_normalized / svg_width) * page_width_mm
                y_mm = (y_normalized / svg_height) * page_height_mm
                
                # STEP 3: Page mm → Robot meters
                x_robot = robot_origin_x + (x_mm * 0.001)
                y_robot = robot_origin_y + (y_mm * 0.001)
                
                points.append((x_robot, y_robot))
            
            if points:
                all_paths.append(points)
        except Exception as e:
            print(f"Warning: Failed to parse path: {e}")
    
    return all_paths


def validate_reachability(robot_paths, controller, z_contact, pitch=0.0, 
                         sample_rate=10, min_threshold=0.90):
    """
    Validate that enough points are reachable using actual IK tests.
    
    Args:
        robot_paths: List of paths with (x, y) coordinates
        controller: Robot controller
        z_contact: Z coordinate for drawing
        pitch: Pitch angle in radians
        sample_rate: Test every Nth point (10 = test 10% of points)
        min_threshold: Minimum fraction of points that must be reachable (0.90 = 90%)
    
    Returns:
        (reachable_fraction, reachable_map)
        reachable_map is a list of lists of bools indicating if each point is reachable
    """
    print(f"\nValidating IK reachability (sampling every {sample_rate} points)...")
    
    total_tested = 0
    total_reachable = 0
    reachable_map = []
    
    for path_idx, path in enumerate(robot_paths):
        path_reachable = []
        for point_idx, (x, y) in enumerate(path):
            # Sample every Nth point for efficiency
            if point_idx % sample_rate == 0:
                reachable, _ = test_ik_reachability(controller, x, y, z_contact, pitch)
                path_reachable.append(reachable)
                total_tested += 1
                if reachable:
                    total_reachable += 1
            else:
                # For non-sampled points, use None (will be checked later if needed)
                path_reachable.append(None)
        
        reachable_map.append(path_reachable)
    
    reachable_fraction = total_reachable / total_tested if total_tested > 0 else 0.0
    
    print(f"  Tested: {total_tested} points")
    print(f"  Reachable: {total_reachable}/{total_tested} ({100*reachable_fraction:.1f}%)")
    
    if reachable_fraction < min_threshold:
        print(f"\n✗ Only {100*reachable_fraction:.1f}% reachable (need >{100*min_threshold:.0f}%)")
        return reachable_fraction, reachable_map
    else:
        print(f"✓ Sufficient coverage ({100*reachable_fraction:.1f}% > {100*min_threshold:.0f}%)")
        return reachable_fraction, reachable_map


def draw_scaled_svg_robust(svg_file, controller, analysis_result, z_contact=0.0, z_safe=0.05,
                          pitch=0.0, min_threshold=0.90):
    """
    Draw SVG with IK validation and graceful handling of unreachable points.
    
    Args:
        min_threshold: Minimum fraction of points that must be reachable (default 0.90 = 90%)
    """
    actual_bounds = analysis_result['actual_bounds']
    page_width_mm, page_height_mm = analysis_result['recommended']
    
    print(f"\nLoading SVG with recommended scaling...")
    
    robot_paths = load_and_scale_svg(
        svg_file,
        actual_bounds,
        page_width_mm,
        page_height_mm,
        robot_origin_x=0.15,
        robot_origin_y=-0.09
    )
    
    print(f"\nLoaded {len(robot_paths)} paths")
    
    # First pass: validate reachability
    reachable_fraction, reachable_map = validate_reachability(
        robot_paths, 
        controller, 
        z_contact, 
        pitch=pitch,
        sample_rate=10,  # Test every 10th point
        min_threshold=min_threshold
    )
    
    if reachable_fraction < min_threshold:
        print(f"\n⚠ Drawing coverage too low!")
        print(f"\nSuggestions:")
        print(f"  1. Move the drawing closer to robot center (adjust origin)")
        print(f"  2. Make the drawing smaller (reduce page_width_mm/height_mm)")
        print(f"  3. Adjust z_contact to be higher (less reach required)")
        
        response = input("\nContinue anyway? (y/n): ")
        if response.lower() != 'y':
            return False
    
    # Draw all paths, skipping unreachable points
    print(f"\nDrawing {len(robot_paths)} paths (skipping unreachable points)...")
    
    total_points = 0
    skipped_points = 0
    drawn_segments = 0
    
    for path_idx, robot_path in enumerate(robot_paths):
        print(f"\n  Path {path_idx+1}/{len(robot_paths)}")
        
        # Build segments of consecutive reachable points
        segments = []
        current_segment = []
        
        for point_idx, (x, y) in enumerate(robot_path):
            total_points += 1
            
            # Test if point is reachable (full check this time)
            reachable, _ = test_ik_reachability(controller, x, y, z_contact, pitch)
            
            if reachable:
                current_segment.append((x, y))
            else:
                skipped_points += 1
                # End current segment if we hit an unreachable point
                if len(current_segment) >= 2:  # Need at least 2 points to draw
                    segments.append(current_segment)
                    drawn_segments += 1
                current_segment = []
        
        # Add final segment if exists
        if len(current_segment) >= 2:
            segments.append(current_segment)
            drawn_segments += 1
        
        # Draw each segment
        for seg_idx, segment in enumerate(segments):
            path = DrawingPath(z_contact=z_contact, z_safe=z_safe)
            
            # Move to start (pen up)
            path.add_point(segment[0][0], segment[0][1], pen_down=False)
            
            # Draw segment (pen down)
            for x, y in segment:
                path.add_point(x, y, pen_down=True)
            
            # Lift pen
            path.add_point(segment[-1][0], segment[-1][1], pen_down=False)
            
            success = path.execute(controller, move_duration=0.3)
            if not success:
                print(f"    Warning: Segment {seg_idx+1} failed, continuing...")
        
        print(f"    Drew {len(segments)} segments, skipped {skipped_points} points so far")
    
    print(f"\n✓ Drawing complete!")
    print(f"  Total points: {total_points}")
    print(f"  Drawn: {total_points - skipped_points} ({100*(total_points-skipped_points)/total_points:.1f}%)")
    print(f"  Skipped: {skipped_points} ({100*skipped_points/total_points:.1f}%)")
    print(f"  Segments: {drawn_segments}")
    
    return True


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python auto_scale_svg_robust.py <svg_file>")
        print("\nExample:")
        print("  python auto_scale_svg_robust.py butterfly.svg")
        sys.exit(1)
    
    svg_file = sys.argv[1]
    
    # Analyze SVG
    analysis = analyze_svg(svg_file)
    
    if not analysis:
        print("\n✗ Failed to analyze SVG")
        sys.exit(1)
    
    # Show recommendation
    print("\n" + "="*70)
    print("RECOMMENDATION")
    print("="*70)
    print("\nUse these parameters:")
    print(f"  actual_bounds = {analysis['actual_bounds']}")
    print(f"  page_width_mm = {analysis['recommended'][0]:.1f}")
    print(f"  page_height_mm = {analysis['recommended'][1]:.1f}")
    
    # Ask to draw
    response = input("\nDraw with these settings? (y/n): ")
    
    if response.lower() != 'y':
        print("\nAnalysis complete. You can manually adjust parameters if needed.")
        sys.exit(0)
    
    # Get z_contact
    z_contact_input = input("\nEnter z_contact (meters, or press Enter for 0.0): ").strip()
    z_contact = float(z_contact_input) if z_contact_input else 0.0
    z_safe = z_contact + 0.05
    
    # Get threshold
    threshold_input = input("\nEnter minimum coverage threshold (0-1, or press Enter for 0.90): ").strip()
    min_threshold = float(threshold_input) if threshold_input else 0.90
    
    # Initialize robot
    print("\nInitializing robot...")
    robot_config = SO100FollowerConfig(
        port="/dev/ttyACM0",
        id="marc",
        use_degrees=True
    )
    robot = SO100Follower(robot_config)
    robot.connect()
    
    if not robot.is_connected:
        raise ValueError("Robot is not connected!")
    
    controller = RobotMovementController(robot)
    
    # Move to safe position
    print("Moving to safe position...")
    controller.move_to_safe_position(duration=3.0)
    
    # Draw
    success = draw_scaled_svg_robust(
        svg_file, 
        controller, 
        analysis, 
        z_contact, 
        z_safe,
        pitch=0.0,
        min_threshold=min_threshold
    )
    
    # Return to safe
    print("\nReturning to safe position...")
    controller.move_to_safe_position(duration=3.0)
    
    if success:
        print("\n✓ Drawing completed successfully!")
    else:
        print("\n✗ Drawing failed or cancelled")


if __name__ == "__main__":
    main()
