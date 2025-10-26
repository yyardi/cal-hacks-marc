#!/usr/bin/env python

"""
Calibration Helper for PaintBot

This tool helps you calibrate your drawing setup by:
1. Finding the optimal z_contact height
2. Setting up the page origin
3. Testing your drawing area boundaries
"""

import math
import time
from robot_movement import RobotMovementController, SO100Follower, SO100FollowerConfig


def find_z_contact(controller: RobotMovementController):
    """
    Interactive tool to find the correct z_contact height with full position control.
    """
    print("\n" + "="*60)
    print("Z_CONTACT CALIBRATION")
    print("="*60)
    print("\nThis will help you find the correct height for pen contact.")
    print("Instructions:")
    print("  1. Place your drawing surface under the robot")
    print("  2. Position a pen in the end effector")
    print("  3. Use controls to position pen above paper")
    print("  4. Lower gradually until pen just touches")
    print("  5. Mark the Z height")
    
    input("\nPress Enter when ready...")
    
    # Start at safe position
    x_current = 0.175
    y_current = 0.0
    z_current = 0.15  # Start high
    pitch_current = 0.0
    step_size = 0.01  # 1cm steps initially
    
    print(f"\nMoving to starting position: x={x_current}, y={y_current}, z={z_current}")
    controller.move_to(x_current, y_current, z_current, pitch_rad=pitch_current, duration=2.0)
    
    print("\n" + "="*60)
    print("MOVEMENT CONTROLS")
    print("="*60)
    print("Position Control:")
    print("  w/s : Forward/Backward (X)")
    print("  a/d : Left/Right (Y)")
    print("  q/e : Up/Down (Z)")
    print("  r/f : Pitch up/down")
    print("\nStep Size:")
    print("  + : Increase step size")
    print("  - : Decrease step size")
    print("  1 : Set to 10mm")
    print("  2 : Set to 1mm")
    print("  3 : Set to 0.1mm")
    print("\nActions:")
    print("  t : Test draw (move in small square)")
    print("  p : Print current position")
    print("  done : Save Z_CONTACT height")
    print("  quit : Cancel")
    print("="*60)
    
    while True:
        response = input(f"\n[Step:{step_size*1000:.1f}mm X:{x_current:.3f} Y:{y_current:.3f} Z:{z_current:.3f}] Command: ").lower().strip()
        
        if response == 'done':
            print(f"\n✓ Z_CONTACT calibrated: {z_current:.3f} m")
            print(f"\nAdd this to your code:")
            print(f"  z_contact = {z_current:.3f}")
            print(f"  z_safe = {z_current + 0.05:.3f}  # 5cm above contact")
            
            # Also print page origin for convenience
            print(f"\nCurrent position (could be page origin):")
            print(f"  page_origin_x = {x_current:.3f}")
            print(f"  page_origin_y = {y_current:.3f}")
            return z_current
            
        elif response == 'quit':
            print("Calibration cancelled")
            return None
            
        elif response == 'p':
            print(f"\nCurrent position:")
            print(f"  X: {x_current:.3f} m ({x_current*1000:.1f} mm)")
            print(f"  Y: {y_current:.3f} m ({y_current*1000:.1f} mm)")
            print(f"  Z: {z_current:.3f} m ({z_current*1000:.1f} mm)")
            print(f"  Pitch: {math.degrees(pitch_current):.1f}°")
            print(f"  Step size: {step_size*1000:.1f} mm")
            continue
            
        elif response == 't':
            print("\nTest draw: small square...")
            test_moves = [
                (x_current + 0.02, y_current, z_current),
                (x_current + 0.02, y_current + 0.02, z_current),
                (x_current, y_current + 0.02, z_current),
                (x_current, y_current, z_current),
            ]
            for x_t, y_t, z_t in test_moves:
                controller.move_to(x_t, y_t, z_t, pitch_rad=pitch_current, duration=0.5)
            print("Test complete")
            continue
            
        # Movement commands
        elif response == 'w':
            x_current += step_size
            print(f"Moving forward to X={x_current:.3f}")
        elif response == 's':
            x_current -= step_size
            print(f"Moving backward to X={x_current:.3f}")
        elif response == 'd':
            y_current += step_size
            print(f"Moving right to Y={y_current:.3f}")
        elif response == 'a':
            y_current -= step_size
            print(f"Moving left to Y={y_current:.3f}")
        elif response == 'q':
            z_current += step_size
            print(f"Moving up to Z={z_current:.3f}")
        elif response == 'e':
            z_current -= step_size
            print(f"Moving down to Z={z_current:.3f}")
        elif response == 'r':
            pitch_current += math.radians(5)
            print(f"Pitch up to {math.degrees(pitch_current):.1f}°")
        elif response == 'f':
            pitch_current -= math.radians(5)
            print(f"Pitch down to {math.degrees(pitch_current):.1f}°")
            
        # Step size adjustment
        elif response == '+':
            step_size = min(step_size * 2, 0.05)  # Max 5cm
            print(f"Step size increased to {step_size*1000:.1f} mm")
            continue
        elif response == '-':
            step_size = max(step_size / 2, 0.0001)  # Min 0.1mm
            print(f"Step size decreased to {step_size*1000:.1f} mm")
            continue
        elif response == '1':
            step_size = 0.01  # 10mm
            print(f"Step size set to 10mm")
            continue
        elif response == '2':
            step_size = 0.001  # 1mm
            print(f"Step size set to 1mm")
            continue
        elif response == '3':
            step_size = 0.0001  # 0.1mm
            print(f"Step size set to 0.1mm")
            continue
        else:
            print("Invalid command. Type 'p' for help.")
            continue
        
        # Execute movement
        success = controller.move_to(x_current, y_current, z_current, pitch_rad=pitch_current, duration=0.5, verbose=False)
        
        if not success:
            print("⚠ Warning: Position may be outside workspace")
            print(f"  Safe ranges: X:[0.15-0.25], Y:[-0.10-0.10], Z:[0.08-0.15]")
            # Revert movement
            if response == 'w':
                x_current -= step_size
            elif response == 's':
                x_current += step_size
            elif response == 'd':
                y_current -= step_size
            elif response == 'a':
                y_current += step_size
            elif response == 'q':
                z_current -= step_size
            elif response == 'e':
                z_current += step_size


def calibrate_page_corners(controller: RobotMovementController):
    """
    Interactive tool to calibrate the drawing area corners.
    """
    print("\n" + "="*60)
    print("PAGE CORNER CALIBRATION")
    print("="*60)
    print("\nThis will help you define your drawing area.")
    print("We'll jog to each corner of your page and record positions.")
    
    input("\nPress Enter when ready...")
    
    corners = {}
    corner_names = [
        ("bottom_left", "Bottom-Left"),
        ("bottom_right", "Bottom-Right"),
        ("top_right", "Top-Right"),
        ("top_left", "Top-Left")
    ]
    
    for corner_id, corner_name in corner_names:
        print("\n" + "-"*60)
        print(f"CALIBRATING: {corner_name} Corner")
        print("-"*60)
        print("Controls:")
        print("  w/s : Forward/Backward (X)")
        print("  a/d : Left/Right (Y)")
        print("  q/e : Up/Down (Z)")
        print("  r/f : Pitch up/down")
        print("  +/- : Increase/decrease step size")
        print("  1/2/3 : Set step to 10mm/1mm/0.1mm")
        print("  p : Print current position")
        print("  done : Save this corner")
        print("  skip : Skip this corner")
        print("-"*60)
        
        # Start from safe position
        controller.move_to_safe_position(duration=2.0)
        
        x_current = 0.175
        y_current = 0.0
        z_current = 0.10
        pitch_current = 0.0
        step = 0.01  # 10mm initial step
        
        while True:
            response = input(f"\n[{corner_name} | Step:{step*1000:.1f}mm | X:{x_current:.3f} Y:{y_current:.3f} Z:{z_current:.3f}] Command: ").lower().strip()
            
            if response == 'done':
                corners[corner_id] = (x_current, y_current, z_current)
                print(f"✓ {corner_name} recorded: ({x_current:.3f}, {y_current:.3f}, {z_current:.3f})")
                break
            elif response == 'skip':
                print(f"⊘ {corner_name} skipped")
                break
            elif response == 'p':
                print(f"\nCurrent position:")
                print(f"  X: {x_current:.3f} m ({x_current*1000:.1f} mm)")
                print(f"  Y: {y_current:.3f} m ({y_current*1000:.1f} mm)")
                print(f"  Z: {z_current:.3f} m ({z_current*1000:.1f} mm)")
                print(f"  Pitch: {math.degrees(pitch_current):.1f}°")
                continue
                
            # Movement commands
            elif response == 'w':
                x_current += step
            elif response == 's':
                x_current -= step
            elif response == 'd':
                y_current += step
            elif response == 'a':
                y_current -= step
            elif response == 'q':
                z_current += step
            elif response == 'e':
                z_current -= step
            elif response == 'r':
                pitch_current += math.radians(5)
            elif response == 'f':
                pitch_current -= math.radians(5)
                
            # Step size adjustment
            elif response == '+':
                step = min(step * 2, 0.05)
                print(f"Step size: {step*1000:.1f} mm")
                continue
            elif response == '-':
                step = max(step / 2, 0.0001)
                print(f"Step size: {step*1000:.1f} mm")
                continue
            elif response == '1':
                step = 0.01
                print("Step size: 10mm")
                continue
            elif response == '2':
                step = 0.001
                print("Step size: 1mm")
                continue
            elif response == '3':
                step = 0.0001
                print("Step size: 0.1mm")
                continue
            else:
                print("Invalid command")
                continue
            
            # Execute movement
            success = controller.move_to(x_current, y_current, z_current, pitch_rad=pitch_current, duration=0.5)
            if not success:
                print("⚠ Position outside workspace - reverted")
                # Revert the movement
                if response == 'w': x_current -= step
                elif response == 's': x_current += step
                elif response == 'd': y_current -= step
                elif response == 'a': y_current += step
                elif response == 'q': z_current -= step
                elif response == 'e': z_current += step
    
    # Calculate drawing area if we have enough corners
    if len(corners) < 2:
        print("\nNot enough corners calibrated. Need at least 2 corners.")
        return corners
    
    print("\n" + "="*60)
    print("CALIBRATION RESULTS")
    print("="*60)
    
    if "bottom_left" in corners and "bottom_right" in corners:
        bl = corners["bottom_left"]
        br = corners["bottom_right"]
        
        if "top_left" in corners:
            tl = corners["top_left"]
            height = math.sqrt((tl[0] - bl[0])**2 + (tl[1] - bl[1])**2)
        elif "top_right" in corners:
            tr = corners["top_right"]
            height = math.sqrt((tr[0] - br[0])**2 + (tr[1] - br[1])**2)
        else:
            height = 0
        
        width = math.sqrt((br[0] - bl[0])**2 + (br[1] - bl[1])**2)
        
        print(f"\nPage dimensions:")
        print(f"  Width:  {width*1000:.1f} mm")
        if height > 0:
            print(f"  Height: {height*1000:.1f} mm")
        
        print(f"\nCorner coordinates:")
        for corner_id, corner_name in corner_names:
            if corner_id in corners:
                x, y, z = corners[corner_id]
                print(f"  {corner_name:15s}: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        print(f"\nAdd this to your PaintBotDrawer:")
        print(f"  page_origin_x = {bl[0]:.3f}")
        print(f"  page_origin_y = {bl[1]:.3f}")
        print(f"  z_contact = {bl[2]:.3f}")
        
        # Calculate rotation if page is rotated
        dx = br[0] - bl[0]
        dy = br[1] - bl[1]
        rotation = math.atan2(dy, dx)
        
        if abs(rotation) > 0.01:  # More than ~0.5 degrees
            print(f"  page_rotation = {rotation:.4f}  # {math.degrees(rotation):.1f}°")
        else:
            print(f"  page_rotation = 0.0  # Page is aligned")
    else:
        print("\nRecorded corners:")
        for corner_id, corner_name in corner_names:
            if corner_id in corners:
                x, y, z = corners[corner_id]
                print(f"  {corner_name:15s}: ({x:.3f}, {y:.3f}, {z:.3f})")
    
    return corners


def quick_workspace_test(controller: RobotMovementController):
    """
    Quick test of common drawing positions.
    """
    print("\n" + "="*60)
    print("QUICK WORKSPACE TEST")
    print("="*60)
    
    test_positions = [
        (0.15, 0.0, 0.10, "Center-low"),
        (0.20, 0.0, 0.10, "Center-mid"),
        (0.20, 0.08, 0.10, "Right"),
        (0.20, -0.08, 0.10, "Left"),
        (0.20, 0.0, 0.13, "High"),
    ]
    
    print("\nTesting common drawing positions...")
    success_count = 0
    
    for x, y, z, name in test_positions:
        result = controller.move_to(x, y, z, pitch_rad=0.0, duration=1.5, wait=True)
        status = "✓" if result else "✗"
        print(f"  {status} {name:12s}: ({x:.3f}, {y:.3f}, {z:.3f})")
        if result:
            success_count += 1
    
    print(f"\nResult: {success_count}/{len(test_positions)} positions reachable")
    
    if success_count == len(test_positions):
        print("✓ All test positions reachable - workspace looks good!")
    elif success_count >= len(test_positions) * 0.8:
        print("⚠ Most positions reachable - adjust unreachable areas")
    else:
        print("✗ Many positions unreachable - check robot setup")


def free_jog_mode(controller: RobotMovementController):
    """
    Free jog mode - move the robot anywhere for exploration or testing.
    """
    print("\n" + "="*60)
    print("FREE JOG MODE")
    print("="*60)
    print("\nMove the robot freely to explore workspace or position pen.")
    
    x_current = 0.175
    y_current = 0.0
    z_current = 0.10
    pitch_current = 0.0
    step = 0.01  # 10mm
    
    print("\n" + "="*60)
    print("CONTROLS")
    print("="*60)
    print("Movement:")
    print("  w/s : Forward/Backward (X)")
    print("  a/d : Left/Right (Y)")
    print("  q/e : Up/Down (Z)")
    print("  r/f : Pitch up/down")
    print("\nStep Size:")
    print("  +/- : Increase/decrease step")
    print("  1/2/3 : Set to 10mm/1mm/0.1mm")
    print("\nActions:")
    print("  p : Print position")
    print("  h : Return to safe/home position")
    print("  t : Test draw (small square)")
    print("  save : Save current position")
    print("  quit : Exit jog mode")
    print("="*60)
    
    saved_positions = []
    
    while True:
        response = input(f"\n[Step:{step*1000:.1f}mm | X:{x_current:.3f} Y:{y_current:.3f} Z:{z_current:.3f}] Command: ").lower().strip()
        
        if response == 'quit':
            print("Exiting jog mode")
            break
            
        elif response == 'h':
            print("Returning to safe position...")
            controller.move_to_safe_position(duration=2.0)
            x_current = 0.175
            y_current = 0.0
            z_current = 0.10
            pitch_current = 0.0
            continue
            
        elif response == 'p':
            print(f"\nCurrent position:")
            print(f"  X: {x_current:.3f} m ({x_current*1000:.1f} mm)")
            print(f"  Y: {y_current:.3f} m ({y_current*1000:.1f} mm)")
            print(f"  Z: {z_current:.3f} m ({z_current*1000:.1f} mm)")
            print(f"  Pitch: {math.degrees(pitch_current):.1f}°")
            print(f"  Step: {step*1000:.1f} mm")
            continue
            
        elif response == 'save':
            saved_positions.append((x_current, y_current, z_current, pitch_current))
            print(f"✓ Position {len(saved_positions)} saved: ({x_current:.3f}, {y_current:.3f}, {z_current:.3f})")
            continue
            
        elif response == 't':
            print("Test draw: 20mm square...")
            test_moves = [
                (x_current + 0.02, y_current, z_current),
                (x_current + 0.02, y_current + 0.02, z_current),
                (x_current, y_current + 0.02, z_current),
                (x_current, y_current, z_current),
            ]
            for x_t, y_t, z_t in test_moves:
                success = controller.move_to(x_t, y_t, z_t, pitch_rad=pitch_current, duration=0.5)
                if not success:
                    print("⚠ Test draw failed - position out of range")
                    break
            print("Test complete")
            continue
            
        # Movement commands
        elif response == 'w':
            x_current += step
        elif response == 's':
            x_current -= step
        elif response == 'd':
            y_current += step
        elif response == 'a':
            y_current -= step
        elif response == 'q':
            z_current += step
        elif response == 'e':
            z_current -= step
        elif response == 'r':
            pitch_current += math.radians(5)
            print(f"Pitch: {math.degrees(pitch_current):.1f}°")
        elif response == 'f':
            pitch_current -= math.radians(5)
            print(f"Pitch: {math.degrees(pitch_current):.1f}°")
            
        # Step size
        elif response == '+':
            step = min(step * 2, 0.05)
            print(f"Step: {step*1000:.1f} mm")
            continue
        elif response == '-':
            step = max(step / 2, 0.0001)
            print(f"Step: {step*1000:.1f} mm")
            continue
        elif response == '1':
            step = 0.01
            print("Step: 10mm")
            continue
        elif response == '2':
            step = 0.001
            print("Step: 1mm")
            continue
        elif response == '3':
            step = 0.0001
            print("Step: 0.1mm")
            continue
        else:
            print("Invalid command. Type 'p' for position, 'h' for home, 'quit' to exit")
            continue
        
        # Execute movement
        success = controller.move_to(x_current, y_current, z_current, pitch_rad=pitch_current, duration=0.5)
        if not success:
            print("⚠ Position outside workspace!")
            # Revert
            if response == 'w': x_current -= step
            elif response == 's': x_current += step
            elif response == 'd': y_current -= step
            elif response == 'a': y_current += step
            elif response == 'q': z_current -= step
            elif response == 'e': z_current += step
    
    # Print saved positions
    if saved_positions:
        print("\n" + "="*60)
        print("SAVED POSITIONS")
        print("="*60)
        for i, (x, y, z, pitch) in enumerate(saved_positions, 1):
            print(f"{i}. X:{x:.3f} Y:{y:.3f} Z:{z:.3f} Pitch:{math.degrees(pitch):.1f}°")


def test_drawing(controller: RobotMovementController):
    """
    Test drawing function to verify z_contact calibration.
    """
    print("\n" + "="*60)
    print("TEST DRAWING")
    print("="*60)
    print("\nThis will draw simple test patterns to verify your calibration.")
    print("Make sure you have:")
    print("  1. Paper positioned under robot")
    print("  2. Pen in end effector")
    print("  3. Calibrated z_contact height")
    
    z_contact_input = input("\nEnter z_contact height (meters, or press Enter for 0.0): ").strip()
    z_contact = float(z_contact_input) if z_contact_input else 0.0
    z_safe = z_contact + 0.05
    
    x_center = 0.20
    y_center = 0.0
    
    print(f"\nUsing z_contact={z_contact:.3f}, z_safe={z_safe:.3f}")
    print(f"Drawing at center position: x={x_center}, y={y_center}")
    
    patterns = {
        '1': ("Small square (20mm)", [
            (x_center, y_center, False),
            (x_center, y_center, True),
            (x_center + 0.02, y_center, True),
            (x_center + 0.02, y_center + 0.02, True),
            (x_center, y_center + 0.02, True),
            (x_center, y_center, True),
            (x_center, y_center, False),
        ]),
        '2': ("Cross (40mm)", [
            (x_center - 0.02, y_center, False),
            (x_center - 0.02, y_center, True),
            (x_center + 0.02, y_center, True),
            (x_center, y_center, False),
            (x_center, y_center - 0.02, True),
            (x_center, y_center + 0.02, True),
            (x_center, y_center, False),
        ]),
        '3': ("Circle (30mm diameter)", None),  # Will be generated
    }
    
    print("\nAvailable patterns:")
    for key, (name, _) in patterns.items():
        print(f"  {key}: {name}")
    
    choice = input("\nSelect pattern (1-3): ").strip()
    
    if choice not in patterns:
        print("Invalid choice")
        return
    
    pattern_name, waypoints = patterns[choice]
    
    if choice == '3':
        # Generate circle
        import math
        radius = 0.015  # 15mm radius = 30mm diameter
        num_points = 20
        waypoints = [(x_center + radius, y_center, False)]  # Move to start
        for i in range(num_points + 1):
            angle = (i / num_points) * 2 * math.pi
            x = x_center + radius * math.cos(angle)
            y = y_center + radius * math.sin(angle)
            waypoints.append((x, y, True))
        waypoints.append((x_center + radius, y_center, False))  # Lift
    
    print(f"\nDrawing {pattern_name}...")
    
    for x, y, pen_down in waypoints:
        z = z_contact if pen_down else z_safe
        success = controller.move_to(x, y, z, pitch_rad=0.0, duration=0.5)
        if not success:
            print("⚠ Drawing failed - position out of range")
            break
    
    print("Test drawing complete!")
    
    # Return to safe position
    controller.move_to_safe_position(duration=2.0)


def main():
    print("="*60)
    print("PAINTBOT CALIBRATION HELPER")
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
    
    # Move to safe position
    print("\nMoving to safe starting position...")
    controller.move_to_safe_position(duration=3.0)
    
    while True:
        print("\n" + "="*60)
        print("CALIBRATION MENU")
        print("="*60)
        print("1. Quick workspace test")
        print("2. Find Z_CONTACT height")
        print("3. Calibrate page corners")
        print("4. Free jog mode")
        print("5. Test drawing")
        print("6. Exit")
        
        choice = input("\nSelect option (1-6): ")
        
        if choice == "1":
            quick_workspace_test(controller)
        elif choice == "2":
            find_z_contact(controller)
        elif choice == "3":
            calibrate_page_corners(controller)
        elif choice == "4":
            free_jog_mode(controller)
        elif choice == "5":
            test_drawing(controller)
        elif choice == "6":
            print("\nReturning to safe position...")
            controller.move_to_safe_position(duration=3.0)
            print("Calibration complete. Goodbye!")
            break
        else:
            print("Invalid choice")


if __name__ == "__main__":
    main()
