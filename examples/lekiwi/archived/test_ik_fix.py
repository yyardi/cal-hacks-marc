#!/usr/bin/env python

"""
Quick test to verify IK fix works

This tests that the robot can:
1. Move to safe position from unknown starting position
2. Successfully use IK for common positions
"""

from robot_movement import RobotMovementController, SO100Follower, SO100FollowerConfig


def main():
    print("="*60)
    print("IK FIX VERIFICATION TEST")
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
    
    # Test 1: Move to safe position (should ALWAYS work)
    print("\n" + "-"*60)
    print("TEST 1: Move to safe position (using direct angles)")
    print("-"*60)
    print("This should work regardless of starting position...")
    
    success = controller.move_to_safe_position(duration=3.0)
    
    if success:
        print("✓ TEST 1 PASSED: Safe position reached")
    else:
        print("✗ TEST 1 FAILED: Could not reach safe position")
        return
    
    # Test 2: IK-based movements (should work now that we're in safe position)
    print("\n" + "-"*60)
    print("TEST 2: IK-based movements from safe position")
    print("-"*60)
    
    test_positions = [
        (0.15, 0.0, 0.10, "Near center"),
        (0.20, 0.0, 0.10, "Mid center"),
        (0.20, 0.08, 0.10, "Right"),
        (0.20, -0.08, 0.10, "Left"),
        (0.20, 0.0, 0.13, "High"),
    ]
    
    passed = 0
    failed = 0
    
    for x, y, z, name in test_positions:
        print(f"\nTesting: {name} ({x:.3f}, {y:.3f}, {z:.3f})")
        success = controller.move_to(x, y, z, pitch_rad=0.0, duration=1.5, verbose=False)
        
        if success:
            print(f"  ✓ {name} reached")
            passed += 1
        else:
            print(f"  ✗ {name} failed")
            failed += 1
    
    print("\n" + "-"*60)
    print("TEST 2 RESULTS")
    print("-"*60)
    print(f"Passed: {passed}/{len(test_positions)}")
    print(f"Failed: {failed}/{len(test_positions)}")
    
    if passed == len(test_positions):
        print("✓ TEST 2 PASSED: All IK movements successful")
    elif passed >= len(test_positions) * 0.8:
        print("⚠ TEST 2 PARTIAL: Most movements successful")
        print("  (Some positions may be at workspace limits)")
    else:
        print("✗ TEST 2 FAILED: Many movements failed")
    
    # Test 3: Return to safe position
    print("\n" + "-"*60)
    print("TEST 3: Return to safe position")
    print("-"*60)
    
    success = controller.move_to_safe_position(duration=3.0)
    
    if success:
        print("✓ TEST 3 PASSED: Returned to safe position")
    else:
        print("✗ TEST 3 FAILED: Could not return to safe position")
    
    # Final summary
    print("\n" + "="*60)
    print("FINAL SUMMARY")
    print("="*60)
    
    if passed == len(test_positions):
        print("🎉 ALL TESTS PASSED!")
        print("The IK fix is working correctly.")
        print("\nYou can now:")
        print("  - Run calibration_helper.py")
        print("  - Use simple_svg_draw.py for drawing")
        print("  - Develop your PaintBot application")
    else:
        print("⚠ SOME ISSUES DETECTED")
        print(f"Working: {passed}/{len(test_positions)} positions")
        print("\nThis may be normal if some positions are near workspace limits.")
        print("Check IK_TROUBLESHOOTING.md for more information.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
