#!/usr/bin/env python

"""
PaintBot Drawing Application Example

This demonstrates how to use the robot_movement library for drawing applications.
Includes examples for:
- Drawing from SVG paths
- Multi-color drawing with pen swaps
- Integration with camera calibration (homography)
"""

import math
import numpy as np
from typing import List, Tuple
from robot_movement import RobotMovementController, DrawingPath, SO100Follower, SO100FollowerConfig


class PaintBotDrawer:
    """
    High-level drawing interface for the PaintBot project.
    Handles coordinate transformations and multi-color drawing.
    """
    
    def __init__(self, controller: RobotMovementController):
        """
        Initialize the PaintBot drawer.
        
        Args:
            controller: RobotMovementController instance
        """
        self.controller = controller
        
        # Drawing parameters (adjust based on your setup)
        self.z_contact = -0.020  # Z height when drawing
        self.z_safe = 0.030    # Z height when traveling
        self.pen_down_duration = 0.3  # Time for pen down movements
        self.pen_up_duration = 0.2    # Time for pen up movements
        
        # Coordinate transformation (page -> robot)
        # These should be calibrated for your specific setup
        self.page_to_robot_scale = 0.001  # mm to meters
        self.page_origin_x = 0.215  # Robot X coordinate of page origin
        self.page_origin_y = -0.075  # Robot Y coordinate of page origin
        self.page_rotation = 1.6991  # Rotation in radians
        
    def page_to_robot_coords(self, x_page: float, y_page: float) -> Tuple[float, float]:
        """
        Convert page coordinates (mm) to robot coordinates (meters).
        
        Args:
            x_page: X coordinate on page in mm
            y_page: Y coordinate on page in mm
            
        Returns:
            (x_robot, y_robot) in meters
        """
        # Apply rotation if needed
        if self.page_rotation != 0:
            cos_r = math.cos(self.page_rotation)
            sin_r = math.sin(self.page_rotation)
            x_rot = x_page * cos_r - y_page * sin_r
            y_rot = x_page * sin_r + y_page * cos_r
        else:
            x_rot, y_rot = x_page, y_page
        
        # Convert to meters and add origin offset
        x_robot = self.page_origin_x + (x_rot * self.page_to_robot_scale)
        y_robot = self.page_origin_y + (y_rot * self.page_to_robot_scale)
        
        return x_robot, y_robot
    
    def draw_svg_path(self, path_points: List[Tuple[float, float]], 
                      pen_down: bool = True) -> bool:
        """
        Draw an SVG path given as a list of (x, y) points in page coordinates (mm).
        
        Args:
            path_points: List of (x, y) tuples in mm
            pen_down: Whether to draw with pen down (True) or just move (False)
            
        Returns:
            True if successful, False otherwise
        """
        if not path_points:
            return True
        
        path = DrawingPath(z_contact=self.z_contact, z_safe=self.z_safe)
        
        # Move to start position with pen up
        x_start, y_start = self.page_to_robot_coords(path_points[0][0], path_points[0][1])
        path.add_point(x_start, y_start, pen_down=False)
        
        # Draw the path
        for x_page, y_page in path_points:
            x_robot, y_robot = self.page_to_robot_coords(x_page, y_page)
            path.add_point(x_robot, y_robot, pen_down=pen_down)
        
        # Lift pen at end
        x_end, y_end = self.page_to_robot_coords(path_points[-1][0], path_points[-1][1])
        path.add_point(x_end, y_end, pen_down=False)
        
        return path.execute(self.controller, move_duration=self.pen_down_duration)
    
    def draw_multiple_paths(self, paths: List[List[Tuple[float, float]]]) -> bool:
        """
        Draw multiple paths in sequence.
        
        Args:
            paths: List of paths, where each path is a list of (x, y) points in mm
            
        Returns:
            True if all paths drawn successfully
        """
        for i, path_points in enumerate(paths):
            print(f"Drawing path {i+1}/{len(paths)} with {len(path_points)} points")
            if not self.draw_svg_path(path_points):
                print(f"Failed to draw path {i+1}")
                return False
        return True
    
    def calibrate_page_origin(self):
        """
        Interactive calibration to set the page origin.
        Jog the robot to the page origin (e.g., bottom-left corner) manually,
        then call this to save the position.
        """
        state = self.controller.get_current_state()
        print("Current robot position:")
        print(f"  Shoulder Pan: {state['shoulder_pan']:.2f}°")
        print(f"  Shoulder Lift: {state['shoulder_lift']:.2f}°")
        print(f"  Elbow Flex: {state['elbow_flex']:.2f}°")
        print(f"  Wrist Flex: {state['wrist_flex']:.2f}°")
        
        # For proper calibration, you'd need to compute x, y from these angles
        # For now, this is a placeholder
        print("\nTo calibrate properly:")
        print("1. Manually jog robot to page origin")
        print("2. Read the x, y, z coordinates")
        print("3. Update page_origin_x and page_origin_y in the code")


def example_draw_text():
    """
    Example: Draw simple text-like strokes.
    """
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
    
    # Create controller and drawer
    controller = RobotMovementController(robot)
    drawer = PaintBotDrawer(controller)
    
    # Move to safe position
    controller.move_to_safe_position(duration=3.0)
    
    # Draw letter "A" (approximate strokes in page coordinates - mm)
    letter_a_paths = [
        [(50, 0), (75, 100)],  # Left leg
        [(75, 100), (100, 0)],  # Right leg
        [(60, 50), (90, 50)]   # Horizontal bar
    ]
    
    print("Drawing letter 'A'")
    drawer.draw_multiple_paths(letter_a_paths)
    
    # Return to safe position
    controller.move_to_safe_position(duration=3.0)


def example_draw_shape_at_scale():
    """
    Example: Draw a shape at a specific scale on A4 paper.
    """
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
    drawer = PaintBotDrawer(controller)
    
    # A4 paper dimensions: 210mm x 297mm
    # Let's draw a simple design in a 100mm x 100mm area
    
    # Star pattern
    star_paths = []
    center_x, center_y = 105, 148.5  # Center of A4
    radius = 40
    
    # 5-pointed star
    for i in range(5):
        angle1 = (i * 2 * math.pi / 5) - math.pi / 2
        angle2 = ((i + 2) * 2 * math.pi / 5) - math.pi / 2
        
        x1 = center_x + radius * math.cos(angle1)
        y1 = center_y + radius * math.sin(angle1)
        x2 = center_x + radius * math.cos(angle2)
        y2 = center_y + radius * math.sin(angle2)
        
        star_paths.append([(x1, y1), (x2, y2)])
    
    # Move to safe position
    controller.move_to_safe_position(duration=3.0)
    
    print("Drawing star pattern")
    drawer.draw_multiple_paths(star_paths)
    
    # Return to safe position
    controller.move_to_safe_position(duration=3.0)


def example_with_homography():
    """
    Example: Use homography matrix for precise camera-based drawing.
    This would integrate with your AprilTag calibration system.
    """
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
    drawer = PaintBotDrawer(controller)
    
    # In a real implementation, you would:
    # 1. Detect AprilTags at page corners using camera
    # 2. Compute homography H: image pixels -> page mm
    # 3. Load your SVG vector art
    # 4. Convert SVG coordinates through H to page coordinates
    # 5. Draw each path
    
    # Placeholder homography matrix (3x3)
    # H = np.array([...])  # From your AprilTag detection
    
    # Example: Convert pixel coordinates to page coordinates
    def pixel_to_page(x_px: float, y_px: float, H: np.ndarray) -> Tuple[float, float]:
        """Apply homography transformation."""
        point = np.array([x_px, y_px, 1.0])
        transformed = H @ point
        x_page = transformed[0] / transformed[2]
        y_page = transformed[1] / transformed[2]
        return x_page, y_page
    
    # Then use drawer.draw_svg_path() with the transformed coordinates
    
    print("For full implementation, integrate with:")
    print("1. Camera + AprilTag detection")
    print("2. SVG path loading and vectorization")
    print("3. Color quantization and multi-pass drawing")
    

def test_workspace_limits():
    """
    Test the robot's reachable workspace to determine safe drawing area.
    """
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
    controller.move_to_safe_position(duration=3.0)
    
    print("\nTesting workspace limits...")
    print("This will probe the reachable workspace of the robot.")
    
    # Test various positions
    test_points = [
        (0.10, 0.00, 0.10, "Near, center"),
        (0.15, 0.00, 0.10, "Medium, center"),
        (0.20, 0.00, 0.10, "Far, center"),
        (0.25, 0.00, 0.10, "Very far, center"),
        (0.15, 0.10, 0.10, "Medium, right"),
        (0.15, -0.10, 0.10, "Medium, left"),
        (0.15, 0.00, 0.15, "Medium, high"),
        (0.15, 0.00, 0.05, "Medium, low"),
        # Additional corner tests for drawing area
        (0.15, 0.10, 0.15, "Right-high corner"),
        (0.15, -0.10, 0.15, "Left-high corner"),
        (0.20, 0.10, 0.10, "Far-right"),
        (0.20, -0.10, 0.10, "Far-left"),
    ]
    
    reachable = []
    unreachable = []
    
    for x, y, z, description in test_points:
        print(f"\nTesting: {description}")
        print(f"  Position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        success = controller.move_to(x, y, z, pitch_rad=0.0, duration=2.0)
        
        if success:
            print("  ✓ Reachable")
            reachable.append((x, y, z, description))
        else:
            print("  ✗ Not reachable")
            unreachable.append((x, y, z, description))
    
    print("\n" + "="*50)
    print("WORKSPACE TEST RESULTS")
    print("="*50)
    print(f"\nReachable positions: {len(reachable)}/{len(test_points)}")
    for x, y, z, desc in reachable:
        print(f"  ✓ {desc}: ({x:.3f}, {y:.3f}, {z:.3f})")
    
    if unreachable:
        print(f"\nUnreachable positions: {len(unreachable)}")
        for x, y, z, desc in unreachable:
            print(f"  ✗ {desc}: ({x:.3f}, {y:.3f}, {z:.3f})")
    
    # Analyze and provide recommendations
    print("\n" + "="*50)
    print("WORKSPACE ANALYSIS & RECOMMENDATIONS")
    print("="*50)
    
    if reachable:
        x_coords = [p[0] for p in reachable]
        y_coords = [p[1] for p in reachable]
        z_coords = [p[2] for p in reachable]
        
        print("\nSafe working ranges:")
        print(f"  X: {min(x_coords):.3f} to {max(x_coords):.3f} m ({(max(x_coords)-min(x_coords))*1000:.0f} mm)")
        print(f"  Y: {min(y_coords):.3f} to {max(y_coords):.3f} m ({(max(y_coords)-min(y_coords))*1000:.0f} mm)")
        print(f"  Z: {min(z_coords):.3f} to {max(z_coords):.3f} m ({(max(z_coords)-min(z_coords))*1000:.0f} mm)")
        
        # Recommended drawing area (slightly conservative)
        x_margin = 0.01
        y_margin = 0.01
        z_margin = 0.01
        
        print("\nRecommended drawing workspace (with safety margins):")
        print(f"  X: {min(x_coords)+x_margin:.3f} to {max(x_coords)-x_margin:.3f} m")
        print(f"  Y: {min(y_coords)+y_margin:.3f} to {max(y_coords)-y_margin:.3f} m")
        print(f"  Z: {min(z_coords)+z_margin:.3f} to {max(z_coords)-z_margin:.3f} m")
        
        # A4 paper comparison
        print("\nFor A4 paper (210mm × 297mm):")
        x_range_mm = (max(x_coords) - min(x_coords) - 2*x_margin) * 1000
        y_range_mm = (max(y_coords) - min(y_coords) - 2*y_margin) * 1000
        
        if x_range_mm >= 210 and y_range_mm >= 297:
            print("  ✓ Full A4 portrait orientation is reachable!")
        elif x_range_mm >= 297 and y_range_mm >= 210:
            print("  ✓ Full A4 landscape orientation is reachable!")
        elif x_range_mm >= 148 and y_range_mm >= 210:
            print("  ✓ A5 portrait orientation is reachable")
        else:
            print(f"  ℹ Available area: {x_range_mm:.0f}mm × {y_range_mm:.0f}mm")
            print(f"    Suitable for partial A4 or smaller drawings")
    
    print("\nNotes:")
    print("  • Unreachable positions at extremes are normal (physical limits)")
    print("  • Stay in the middle of workspace for best reliability")
    print("  • Test with actual drawing surface height for z_contact calibration")
    
    # Return to safe position
    controller.move_to_safe_position(duration=3.0)


if __name__ == "__main__":
    import sys
    
    print("PaintBot Drawing Application Examples")
    print("="*50)
    print("\nAvailable examples:")
    print("  1. Test workspace limits (recommended first)")
    print("  2. Draw text (letter 'A')")
    print("  3. Draw shape at scale (star)")
    print("  4. Homography integration info")
    
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("\nEnter choice (1-4): ")
    
    if choice == "1":
        test_workspace_limits()
    elif choice == "2":
        example_draw_text()
    elif choice == "3":
        example_draw_shape_at_scale()
    elif choice == "4":
        example_with_homography()
    else:
        print("Invalid choice. Run with argument 1, 2, 3, or 4")
