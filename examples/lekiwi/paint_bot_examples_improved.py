#!/usr/bin/env python

"""
PaintBot Drawing Application - Improved Version with Z-Height Calibration

This version adds:
- Interactive Z-height calibration to find the paper surface
- Customizable starting position for drawings
- Step-by-step Z adjustment with keyboard control
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
    
    def __init__(self, controller: RobotMovementController, z_contact: float = -0.020, z_safe: float = 0.030):
        """
        Initialize the PaintBot drawer.
        
        Args:
            controller: RobotMovementController instance
            z_contact: Z height when drawing (can be adjusted via calibration)
            z_safe: Z height when traveling
        """
        self.controller = controller
        
        # Drawing parameters (adjust based on your setup)
        self.z_contact = z_contact  # Z height when drawing - now adjustable
        self.z_safe = z_safe        # Z height when traveling
        self.pen_down_duration = 0.3  # Time for pen down movements
        self.pen_up_duration = 0.2    # Time for pen up movements
        
        # Coordinate transformation (page -> robot)
        # These should be calibrated for your specific setup
        self.page_to_robot_scale = 0.001  # mm to meters
        self.page_origin_x = 0.215  # Robot X coordinate of page origin
        self.page_origin_y = -0.075  # Robot Y coordinate of page origin
        self.page_rotation = 1.6991  # Rotation in radians
        
    def set_z_contact(self, z_contact: float):
        """Update the Z contact height."""
        self.z_contact = z_contact
        print(f"Z contact height set to: {z_contact:.4f} m")
    
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


def calibrate_z_height_interactive(controller: RobotMovementController, 
                                   start_x: float = 0.15, 
                                   start_y: float = 0.0) -> float:
    """
    Interactive Z-height calibration. Use keyboard to lower the pen until it touches paper.
    
    Args:
        controller: RobotMovementController instance
        start_x: X position to calibrate at (meters)
        start_y: Y position to calibrate at (meters)
        
    Returns:
        Calibrated z_contact value
    """
    print("\n" + "="*60)
    print("Z-HEIGHT CALIBRATION MODE")
    print("="*60)
    print("\nThis will help you find the correct Z height for drawing.")
    print("\nControls:")
    print("  Q/E: Move Z up/down by 0.001m (1mm)")
    print("  W/S: Move Z up/down by 0.0001m (0.1mm) - fine adjustment")
    print("  T: Test current height by moving pen down and up")
    print("  X: Finish calibration and return value")
    print("  C: Cancel and return default")
    print("\nGoal: Lower until pen barely touches paper")
    print("="*60)
    
    # Move to starting position
    print(f"\nMoving to calibration position: x={start_x:.3f}, y={start_y:.3f}")
    current_z = 0.10  # Start high
    controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=2.0)
    
    try:
        # Import keyboard control (use the teleoperate.py approach)
        from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
        
        keyboard_config = KeyboardTeleopConfig(id="calibration_keyboard")
        keyboard = KeyboardTeleop(keyboard_config)
        keyboard.connect()
        
        if not keyboard.is_connected:
            print("Warning: Keyboard not connected. Using manual input mode.")
            return manual_z_calibration(controller, start_x, start_y, current_z)
        
        print(f"\nCurrent Z height: {current_z:.4f} m")
        print("Start adjusting with Q/E (large) or W/S (fine)...")
        
        while True:
            keyboard_action = keyboard.get_action()
            
            if keyboard_action:
                for key in keyboard_action.keys():
                    if key == 'q':  # Up large
                        current_z += 0.05
                        print(f"Z = {current_z:.4f} m (↑ 1mm)")
                        controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                    
                    elif key == 'e':  # Down large
                        current_z -= 0.05
                        print(f"Z = {current_z:.4f} m (↓ 1mm)")
                        controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                    
                    elif key == 'w':  # Up fine
                        current_z += 0.01
                        print(f"Z = {current_z:.4f} m (↑ 0.1mm)")
                        controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                    
                    elif key == 's':  # Down fine
                        current_z -= 0.01
                        print(f"Z = {current_z:.4f} m (↓ 0.1mm)")
                        controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                    
                    elif key == 't':  # Test
                        print("Testing current height...")
                        # Move up, then down to current_z, then back up
                        controller.move_to(start_x, start_y, current_z + 0.02, pitch_rad=0.0, duration=0.5)
                        controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                        controller.move_to(start_x, start_y, current_z + 0.02, pitch_rad=0.0, duration=0.5)
                        print(f"Current Z = {current_z:.4f} m")
                    
                    elif key == 'x':  # Accept
                        print(f"\n✓ Z height calibrated: {current_z:.4f} m")
                        # Move up to safe height
                        controller.move_to(start_x, start_y, 0.10, pitch_rad=0.0, duration=1.0)
                        return current_z
                    
                    elif key == 'c':  # Cancel
                        print("\n✗ Calibration cancelled")
                        controller.move_to(start_x, start_y, 0.10, pitch_rad=0.0, duration=1.0)
                        return -0.020  # Default value
    
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted")
        controller.move_to(start_x, start_y, 0.10, pitch_rad=0.0, duration=1.0)
        return current_z
    
    except Exception as e:
        print(f"\nError during calibration: {e}")
        print("Falling back to manual input mode...")
        return manual_z_calibration(controller, start_x, start_y, current_z)


def manual_z_calibration(controller: RobotMovementController, 
                         start_x: float, start_y: float, current_z: float) -> float:
    """
    Manual Z-height calibration using text input.
    """
    print("\nManual Z-height adjustment mode")
    print("Commands: +1 (up 1mm), -1 (down 1mm), +0.1 (up 0.1mm), -0.1 (down 0.1mm)")
    print("          test (test current height), done (finish)")
    
    while True:
        try:
            cmd = input(f"\nZ={current_z:.4f}m > ").strip().lower()
            
            if cmd == 'done' or cmd == 'x':
                controller.move_to(start_x, start_y, 0.10, pitch_rad=0.0, duration=1.0)
                return current_z
            
            elif cmd == 'test' or cmd == 't':
                print("Testing...")
                controller.move_to(start_x, start_y, current_z + 0.02, pitch_rad=0.0, duration=0.5)
                controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                controller.move_to(start_x, start_y, current_z + 0.02, pitch_rad=0.0, duration=0.5)
            
            elif cmd in ['+1', '+']:
                current_z += 0.001
                controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                print(f"Z = {current_z:.4f}m")
            
            elif cmd in ['-1', '-']:
                current_z -= 0.001
                controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                print(f"Z = {current_z:.4f}m")
            
            elif cmd == '+0.1':
                current_z += 0.0001
                controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                print(f"Z = {current_z:.4f}m")
            
            elif cmd == '-0.1':
                current_z -= 0.0001
                controller.move_to(start_x, start_y, current_z, pitch_rad=0.0, duration=0.5)
                print(f"Z = {current_z:.4f}m")
            
            else:
                print("Unknown command. Use: +1, -1, +0.1, -0.1, test, done")
        
        except KeyboardInterrupt:
            print("\nInterrupted")
            controller.move_to(start_x, start_y, 0.10, pitch_rad=0.0, duration=1.0)
            return current_z


def draw_star_at_position(controller: RobotMovementController,
                          center_x_robot: float,
                          center_y_robot: float,
                          radius_meters: float = 0.04,
                          z_contact: float = -0.020,
                          z_safe: float = 0.030,
                          num_points: int = 5):
    """
    Draw a star pattern centered at a specific robot coordinate position.
    
    Args:
        controller: RobotMovementController instance
        center_x_robot: Center X in robot coordinates (meters)
        center_y_robot: Center Y in robot coordinates (meters)
        radius_meters: Star radius in meters (default 40mm = 0.04m)
        z_contact: Z height when drawing
        z_safe: Z height when traveling
        num_points: Number of star points (default 5)
    """
    print(f"\nDrawing {num_points}-pointed star at position:")
    print(f"  Center: ({center_x_robot:.3f}, {center_y_robot:.3f}) meters")
    print(f"  Radius: {radius_meters*1000:.1f} mm")
    print(f"  Z contact: {z_contact:.4f} m")
    
    # Generate star points in robot coordinates
    star_segments = []
    for i in range(num_points):
        angle1 = (i * 2 * math.pi / num_points) - math.pi / 2
        angle2 = ((i + 2) * 2 * math.pi / num_points) - math.pi / 2
        
        x1 = center_x_robot + radius_meters * math.cos(angle1)
        y1 = center_y_robot + radius_meters * math.sin(angle1)
        x2 = center_x_robot + radius_meters * math.cos(angle2)
        y2 = center_y_robot + radius_meters * math.sin(angle2)
        
        star_segments.append([(x1, y1), (x2, y2)])
    
    # Draw each segment
    from robot_movement import DrawingPath
    
    for i, segment in enumerate(star_segments):
        print(f"  Drawing segment {i+1}/{len(star_segments)}")
        
        path = DrawingPath(z_contact=z_contact, z_safe=z_safe)
        
        # Move to start with pen up
        path.add_point(segment[0][0], segment[0][1], pen_down=False)
        
        # Draw line with pen down
        path.add_point(segment[1][0], segment[1][1], pen_down=True)
        
        # Lift pen
        path.add_point(segment[1][0], segment[1][1], pen_down=False)
        
        # Execute this segment
        if not path.execute(controller, move_duration=0.3):
            print(f"  Warning: Segment {i+1} may have failed")
    
    print("Star drawing complete!")


def interactive_star_demo():
    """
    Interactive demo that lets you:
    1. Calibrate Z height
    2. Specify star center position
    3. Draw the star
    """
    print("\n" + "="*60)
    print("INTERACTIVE STAR DRAWING DEMO")
    print("="*60)
    
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
    
    # Move to safe position first
    print("\nMoving to safe position...")
    controller.move_to_safe_position(duration=3.0)
    
    # Step 1: Calibrate Z height
    print("\n" + "="*60)
    print("STEP 1: Calibrate Z Height")
    print("="*60)
    calibrate = input("Would you like to calibrate Z height? (y/n): ").strip().lower()
    
    if calibrate == 'y':
        # Ask for calibration position
        print("\nWhere should we calibrate? (This will be roughly the center of your drawing)")
        try:
            cal_x = float(input("  X position (meters, default 0.15): ") or "0.15")
            cal_y = float(input("  Y position (meters, default 0.0): ") or "0.0")
        except ValueError:
            print("Invalid input, using defaults")
            cal_x, cal_y = 0.15, 0.0
        
        z_contact = calibrate_z_height_interactive(controller, cal_x, cal_y)
    else:
        z_contact = -0.020
        print(f"Using default Z contact: {z_contact:.4f} m")
    
    z_safe = z_contact + 0.05  # 50mm above contact
    
    # Step 2: Get star parameters
    print("\n" + "="*60)
    print("STEP 2: Star Parameters")
    print("="*60)
    
    try:
        center_x = float(input("Center X position (meters, default 0.15): ") or "0.15")
        center_y = float(input("Center Y position (meters, default 0.0): ") or "0.0")
        radius = float(input("Star radius (meters, default 0.04 = 40mm): ") or "0.04")
        num_points = int(input("Number of star points (default 5): ") or "5")
    except ValueError:
        print("Invalid input, using defaults")
        center_x, center_y, radius, num_points = 0.15, 0.0, 0.04, 5
    
    # Step 3: Confirm and draw
    print("\n" + "="*60)
    print("STEP 3: Draw Star")
    print("="*60)
    print(f"Ready to draw {num_points}-pointed star:")
    print(f"  Center: ({center_x:.3f}, {center_y:.3f}) m")
    print(f"  Radius: {radius*1000:.1f} mm")
    print(f"  Z contact: {z_contact:.4f} m")
    print(f"  Z safe: {z_safe:.4f} m")
    
    proceed = input("\nProceed with drawing? (y/n): ").strip().lower()
    
    if proceed == 'y':
        draw_star_at_position(
            controller,
            center_x,
            center_y,
            radius,
            z_contact,
            z_safe,
            num_points
        )
    else:
        print("Drawing cancelled")
    
    # Return to safe position
    print("\nReturning to safe position...")
    controller.move_to_safe_position(duration=3.0)
    print("Done!")


def quick_star_demo():
    """
    Quick demo - draws a star at the default position with default Z height.
    Good for testing after you've already calibrated.
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
    
    # Parameters (adjust these based on your calibration)
    center_x = 0.15  # meters
    center_y = 0.0   # meters
    radius = 0.04    # 40mm
    z_contact = -0.020  # Adjust this based on your calibration!
    z_safe = 0.030
    
    print("Quick star demo - using default parameters")
    print(f"Center: ({center_x}, {center_y}), Radius: {radius*1000}mm")
    print(f"Z contact: {z_contact}m")
    print("\nMoving to safe position...")
    
    controller.move_to_safe_position(duration=3.0)
    
    draw_star_at_position(
        controller,
        center_x,
        center_y,
        radius,
        z_contact,
        z_safe,
        num_points=5
    )
    
    controller.move_to_safe_position(duration=3.0)
    print("Done!")


if __name__ == "__main__":
    import sys
    
    print("PaintBot Drawing Application - Improved Version")
    print("="*60)
    print("\nAvailable modes:")
    print("  1. Interactive star demo (with Z calibration)")
    print("  2. Quick star demo (uses saved parameters)")
    print("  3. Z-height calibration only")
    
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("\nEnter choice (1-3): ")
    
    if choice == "1":
        interactive_star_demo()
    elif choice == "2":
        quick_star_demo()
    elif choice == "3":
        robot_config = SO100FollowerConfig(
            port="/dev/ttyACM0",
            id="marc",
            use_degrees=True
        )
        robot = SO100Follower(robot_config)
        robot.connect()
        if robot.is_connected:
            controller = RobotMovementController(robot)
            controller.move_to_safe_position(duration=3.0)
            z_height = calibrate_z_height_interactive(controller)
            print(f"\n✓ Calibrated Z height: {z_height:.4f} m")
            print("Use this value in your drawing code!")
    else:
        print("Invalid choice. Run with argument 1, 2, or 3")
