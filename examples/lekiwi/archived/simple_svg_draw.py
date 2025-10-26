#!/usr/bin/env python

"""
Simple SVG Drawing Example

Shows how to use your existing svg_path_loading.py with the robot system.
"""

from xml.dom import minidom
from svgpathtools import parse_path
from robot_movement import RobotMovementController, DrawingPath, SO100Follower, SO100FollowerConfig


def load_svg_paths(svg_file):
    """Your existing SVG loader (from svg_path_loading.py)"""
    doc = minidom.parse(svg_file)
    paths = []
    for path in doc.getElementsByTagName('path'):
        d = path.getAttribute('d')
        svg_path = parse_path(d)
        points = [(seg.start.real, seg.start.imag) for seg in svg_path]
        if points:
            paths.append(points)
    return paths


def svg_pixels_to_robot_coords(x_svg, y_svg, 
                               svg_width=500, svg_height=500,
                               page_width_mm=200, page_height_mm=200,
                               robot_origin_x=0.15, robot_origin_y=-0.10):
    """
    Convert SVG pixel coordinates to robot coordinates.
    
    Pipeline: SVG pixels → Page mm → Robot meters
    
    Args:
        x_svg, y_svg: Coordinates in SVG pixels
        svg_width, svg_height: SVG canvas size in pixels
        page_width_mm, page_height_mm: Physical drawing area in mm
        robot_origin_x, robot_origin_y: Robot coords of page origin in meters
        
    Returns:
        (x_robot, y_robot) in meters
    """
    # Step 1: SVG pixels → Page millimeters
    x_mm = (x_svg / svg_width) * page_width_mm
    y_mm = (y_svg / svg_height) * page_height_mm
    
    # Step 2: Page millimeters → Robot meters
    x_robot = robot_origin_x + (x_mm * 0.001)  # mm to meters
    y_robot = robot_origin_y + (y_mm * 0.001)
    
    return x_robot, y_robot


def draw_svg_simple(svg_file, controller, 
                   svg_width=500, svg_height=500,
                   page_width_mm=200, page_height_mm=200):
    """
    Draw an SVG file using your existing loader and the robot controller.
    
    Args:
        svg_file: Path to SVG file
        controller: RobotMovementController instance
        svg_width, svg_height: SVG dimensions in pixels (from viewBox)
        page_width_mm, page_height_mm: Physical size to scale to
    """
    # Load SVG paths using your existing function
    svg_paths = load_svg_paths(svg_file)
    print(f"Loaded {len(svg_paths)} paths from {svg_file}")
    
    # Drawing settings
    z_contact = 0.0  # Adjust based on your calibration
    z_safe = 0.05
    robot_origin_x = 0.15  # Adjust to your calibrated page origin
    robot_origin_y = -0.10
    
    # Convert and draw each path
    for i, svg_path_points in enumerate(svg_paths):
        print(f"\nDrawing path {i+1}/{len(svg_paths)} ({len(svg_path_points)} points)")
        
        # Create a DrawingPath
        path = DrawingPath(z_contact=z_contact, z_safe=z_safe)
        
        # Convert all points to robot coordinates
        robot_points = []
        for x_svg, y_svg in svg_path_points:
            x_robot, y_robot = svg_pixels_to_robot_coords(
                x_svg, y_svg,
                svg_width, svg_height,
                page_width_mm, page_height_mm,
                robot_origin_x, robot_origin_y
            )
            robot_points.append((x_robot, y_robot))
        
        # Check if all points are reachable
        all_reachable = True
        for x_robot, y_robot in robot_points:
            if not (0.15 <= x_robot <= 0.25 and -0.10 <= y_robot <= 0.10):
                print(f"  Warning: Point ({x_robot:.3f}, {y_robot:.3f}) outside workspace!")
                all_reachable = False
                break
        
        if not all_reachable:
            print(f"  Skipping path {i+1} (outside workspace)")
            continue
        
        # Build the drawing path
        # Move to start with pen up
        path.add_point(robot_points[0][0], robot_points[0][1], pen_down=False)
        
        # Draw the path with pen down
        for x_robot, y_robot in robot_points:
            path.add_point(x_robot, y_robot, pen_down=True)
        
        # Lift pen
        path.add_point(robot_points[-1][0], robot_points[-1][1], pen_down=False)
        
        # Execute the path
        success = path.execute(controller, move_duration=0.3)
        if not success:
            print(f"  Failed to complete path {i+1}")
            return False
    
    print("\nDrawing complete!")
    return True


def main():
    """
    Main drawing program - simplified version of your svg_path_loading.py
    """
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python simple_svg_draw.py <svg_file> [svg_width] [svg_height]")
        print("\nExamples:")
        print("  python simple_svg_draw.py butterfly.svg")
        print("  python simple_svg_draw.py butterfly.svg 500 500")
        sys.exit(1)
    
    svg_file = sys.argv[1]
    svg_width = int(sys.argv[2]) if len(sys.argv) > 2 else 500
    svg_height = int(sys.argv[3]) if len(sys.argv) > 3 else 500
    
    print("="*60)
    print("SIMPLE SVG DRAWING")
    print("="*60)
    print(f"\nSVG File: {svg_file}")
    print(f"SVG Size: {svg_width}px × {svg_height}px")
    print(f"Drawing Area: 200mm × 200mm")
    
    # Initialize robot
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
    
    # Preview the paths
    print("\nLoading paths...")
    svg_paths = load_svg_paths(svg_file)
    print(f"Found {len(svg_paths)} paths")
    
    # Show first few points of first path
    if svg_paths:
        print(f"\nFirst path preview (first 3 points):")
        for i, (x, y) in enumerate(svg_paths[0][:3]):
            x_robot, y_robot = svg_pixels_to_robot_coords(
                x, y, svg_width, svg_height, 
                page_width_mm=200, page_height_mm=200
            )
            print(f"  SVG ({x:.1f}, {y:.1f}) → Robot ({x_robot:.3f}, {y_robot:.3f})")
    
    # Ask to continue
    response = input("\nStart drawing? (y/n): ")
    
    if response.lower() != 'y':
        print("Cancelled")
        return
    
    # Move to safe position
    print("\nMoving to safe position...")
    controller.move_to_safe_position(duration=3.0)
    
    # Draw the SVG
    success = draw_svg_simple(
        svg_file, 
        controller,
        svg_width=svg_width,
        svg_height=svg_height,
        page_width_mm=200,  # Adjust to fit your workspace
        page_height_mm=200
    )
    
    # Return to safe position
    print("\nReturning to safe position...")
    controller.move_to_safe_position(duration=3.0)
    
    if success:
        print("\n✓ Drawing completed successfully!")
    else:
        print("\n✗ Drawing failed")


if __name__ == "__main__":
    main()
