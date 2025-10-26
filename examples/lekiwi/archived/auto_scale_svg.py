#!/usr/bin/env python

"""
SVG Auto-Scaler for Robot Workspace

Automatically detects SVG dimensions and scales to fit the robot's workspace.
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
    
    if min_x is not None:
        actual_width = max_x - min_x
        actual_height = max_y - min_y
        
        print(f"\nActual Path Bounds:")
        print(f"  X range: {min_x:.1f} to {max_x:.1f}")
        print(f"  Y range: {min_y:.1f} to {max_y:.1f}")
        print(f"  Size: {actual_width:.1f} × {actual_height:.1f} px")
        
        # Use actual bounds if viewBox not available
        if not vb_width:
            vb_width = actual_width
            vb_height = actual_height
            vb_min_x = min_x
            vb_min_y = min_y
            print(f"\n⚠ Using actual bounds as viewBox")
    else:
        print(f"\n✗ Could not find any paths in SVG!")
        return None
    
    # Robot workspace
    robot_x_range = 0.10  # 100mm (0.15 to 0.25)
    robot_y_range = 0.20  # 200mm (-0.10 to 0.10)
    
    print(f"\nRobot Workspace:")
    print(f"  X: 0.15 to 0.25 m (100mm range)")
    print(f"  Y: -0.10 to 0.10 m (200mm range)")
    print(f"  Safe drawing area: ~80mm × 180mm (with margins)")
    
    # Calculate scaling options
    aspect_ratio = vb_width / vb_height if vb_height > 0 else 1.0
    
    print(f"\nSVG Aspect Ratio: {aspect_ratio:.2f}")
    
    # Option 1: Fit width to robot X (100mm with margin)
    target_width_mm_1 = 80  # Conservative
    scale_1 = target_width_mm_1 / vb_width if vb_width > 0 else 1
    scaled_height_1 = vb_height * scale_1
    
    # Option 2: Fit height to robot Y (180mm with margin)  
    target_height_mm_2 = 150
    scale_2 = target_height_mm_2 / vb_height if vb_height > 0 else 1
    scaled_width_2 = vb_width * scale_2
    
    # Option 3: Fit both (use smaller scale factor)
    scale_x = 80 / vb_width if vb_width > 0 else 1
    scale_y = 150 / vb_height if vb_height > 0 else 1
    scale_3 = min(scale_x, scale_y)
    scaled_width_3 = vb_width * scale_3
    scaled_height_3 = vb_height * scale_3
    
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
        'svg_width': vb_width,
        'svg_height': vb_height,
        'svg_min_x': vb_min_x,
        'svg_min_y': vb_min_y,
        'option1': (target_width_mm_1, scaled_height_1),
        'option2': (scaled_width_2, target_height_mm_2),
        'option3': (scaled_width_3, scaled_height_3),
        'recommended': (scaled_width_3, scaled_height_3)
    }


def load_and_scale_svg(svg_file, svg_width, svg_height, page_width_mm, page_height_mm,
                       robot_origin_x=0.15, robot_origin_y=-0.09):
    """
    Load SVG and transform to robot coordinates with proper scaling.
    
    Args:
        svg_file: Path to SVG file
        svg_width: SVG viewBox width in pixels
        svg_height: SVG viewBox height in pixels
        page_width_mm: Physical width to scale to in mm
        page_height_mm: Physical height to scale to in mm
        robot_origin_x: Robot X coordinate of page origin
        robot_origin_y: Robot Y coordinate of page origin
    
    Returns:
        List of paths with robot coordinates
    """
    from xml.dom import minidom
    from svgpathtools import parse_path
    
    doc = minidom.parse(svg_file)
    all_paths = []
    
    print(f"\nTransformation:")
    print(f"  SVG: {svg_width:.0f} × {svg_height:.0f} px")
    print(f"  Page: {page_width_mm:.1f} × {page_height_mm:.1f} mm")
    print(f"  Origin: ({robot_origin_x:.3f}, {robot_origin_y:.3f}) m")
    
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
                
                # SVG pixels → Page mm
                x_mm = (x_svg / svg_width) * page_width_mm
                y_mm = (y_svg / svg_height) * page_height_mm
                
                # Page mm → Robot meters
                x_robot = robot_origin_x + (x_mm * 0.001)
                y_robot = robot_origin_y + (y_mm * 0.001)
                
                points.append((x_robot, y_robot))
            
            if points:
                all_paths.append(points)
        except Exception as e:
            print(f"Warning: Failed to parse path: {e}")
    
    return all_paths


def draw_scaled_svg(svg_file, controller, analysis_result, z_contact=0.0, z_safe=0.05):
    """
    Draw SVG with proper scaling based on analysis.
    """
    svg_width = analysis_result['svg_width']
    svg_height = analysis_result['svg_height']
    page_width_mm, page_height_mm = analysis_result['recommended']
    
    print(f"\nLoading SVG with recommended scaling...")
    
    robot_paths = load_and_scale_svg(
        svg_file,
        svg_width,
        svg_height,
        page_width_mm,
        page_height_mm,
        robot_origin_x=0.15,
        robot_origin_y=-0.09
    )
    
    print(f"\nLoaded {len(robot_paths)} paths")
    
    # Check if all points are in workspace
    in_workspace = 0
    out_workspace = 0
    
    for path in robot_paths:
        for x, y in path:
            if 0.15 <= x <= 0.25 and -0.10 <= y <= 0.10:
                in_workspace += 1
            else:
                out_workspace += 1
    
    total_points = in_workspace + out_workspace
    print(f"\nWorkspace check:")
    print(f"  Points in workspace: {in_workspace}/{total_points} ({100*in_workspace/total_points:.1f}%)")
    print(f"  Points outside: {out_workspace}/{total_points}")
    
    if out_workspace > 0:
        print(f"\n⚠ Warning: {out_workspace} points outside workspace!")
        print("Consider:")
        print("  1. Adjusting page_origin (robot_origin_x/y)")
        print("  2. Using smaller page size")
        print("  3. Centering the drawing")
        
        response = input("\nContinue anyway? (y/n): ")
        if response.lower() != 'y':
            return False
    
    # Draw all paths
    print(f"\nDrawing {len(robot_paths)} paths...")
    
    for i, robot_path in enumerate(robot_paths):
        print(f"  Path {i+1}/{len(robot_paths)} ({len(robot_path)} points)")
        
        path = DrawingPath(z_contact=z_contact, z_safe=z_safe)
        
        # Move to start (pen up)
        path.add_point(robot_path[0][0], robot_path[0][1], pen_down=False)
        
        # Draw path (pen down)
        for x, y in robot_path:
            path.add_point(x, y, pen_down=True)
        
        # Lift pen
        path.add_point(robot_path[-1][0], robot_path[-1][1], pen_down=False)
        
        success = path.execute(controller, move_duration=0.3)
        if not success:
            print(f"  Failed to complete path {i+1}")
            return False
    
    print("\n✓ Drawing complete!")
    return True


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python auto_scale_svg.py <svg_file>")
        print("\nExample:")
        print("  python auto_scale_svg.py butterfly.svg")
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
    print(f"  svg_width = {analysis['svg_width']:.0f}")
    print(f"  svg_height = {analysis['svg_height']:.0f}")
    print(f"  page_width_mm = {analysis['recommended'][0]:.1f}")
    print(f"  page_height_mm = {analysis['recommended'][1]:.1f}")
    
    # Ask to draw
    response = input("\nDraw with these settings? (y/n): ")
    
    if response.lower() != 'y':
        print("\nYou can use these values with simple_svg_draw.py:")
        print(f"  python simple_svg_draw.py {svg_file} {analysis['svg_width']:.0f} {analysis['svg_height']:.0f}")
        print(f"  Then modify page_width_mm={analysis['recommended'][0]:.1f}")
        sys.exit(0)
    
    # Get z_contact
    z_contact_input = input("\nEnter z_contact (meters, or press Enter for 0.0): ").strip()
    z_contact = float(z_contact_input) if z_contact_input else 0.0
    z_safe = z_contact + 0.05
    
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
    success = draw_scaled_svg(svg_file, controller, analysis, z_contact, z_safe)
    
    # Return to safe
    print("\nReturning to safe position...")
    controller.move_to_safe_position(duration=3.0)
    
    if success:
        print("\n✓ Drawing completed successfully!")
    else:
        print("\n✗ Drawing failed or cancelled")


if __name__ == "__main__":
    main()
