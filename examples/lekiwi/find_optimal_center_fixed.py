#!/usr/bin/env python

"""
Automated Workspace Center Finder (CORRECTED DEFAULTS)

This tool scans through possible paper center positions and finds
the location where your drawing has maximum reachability.

FIXED: Proper default workspace bounds
"""

import math
import time
import sys
from xml.dom import minidom
from svgpathtools import parse_path
from robot_movement import RobotMovementController, SO100Follower, SO100FollowerConfig

# Import calibration functions
from workspace_calibration import save_calibration

# Import IK function
sys.path.insert(0, '/mnt/project')
from teleoperate import xyzp_inverse_kinematics


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


def load_and_scale_svg_from_center(svg_file, actual_bounds, page_width_mm, page_height_mm,
                                    center_x, center_y):
    """
    Load SVG and transform to robot coordinates using center position.
    
    Returns sampled points for reachability testing (10% of full path).
    """
    min_x, min_y, max_x, max_y = actual_bounds
    svg_width = max_x - min_x
    svg_height = max_y - min_y
    
    # Calculate origin from center
    page_width_m = page_width_mm * 0.001
    page_height_m = page_height_mm * 0.001
    robot_origin_x = center_x - page_width_m / 2
    robot_origin_y = center_y - page_height_m / 2
    
    doc = minidom.parse(svg_file)
    all_points = []
    
    for path_elem in doc.getElementsByTagName('path'):
        d = path_elem.getAttribute('d')
        if not d:
            continue
        
        try:
            svg_path = parse_path(d)
            
            # Sample 10% of points for speed
            num_samples = max(3, len(svg_path))
            for i in range(0, num_samples, max(1, num_samples // 10)):
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
                
                all_points.append((x_robot, y_robot))
        except Exception:
            pass
    
    return all_points


def test_reachability_fast(robot, points, z_contact, pitch=0.0):
    """
    Test how many points in a list are reachable.
    
    Returns:
        tuple: (reachable_count, total_count, reachable_fraction)
    """
    reachable = 0
    total = len(points)
    
    for x, y in points:
        success, _ = xyzp_inverse_kinematics(robot, x, y, z_contact, pitch)
        if success:
            reachable += 1
    
    fraction = reachable / total if total > 0 else 0
    return reachable, total, fraction


def scan_workspace_grid(robot, svg_file, analysis, page_width_mm, page_height_mm,
                       z_contact, pitch=0.0,
                       x_min=0.17, x_max=0.25, x_step=0.01,
                       y_min=-0.04, y_max=0.09, y_step=0.01):
    """
    Scan a grid of possible center positions and find the best one.
    
    CORRECTED DEFAULTS:
    - x_max = 0.25 (was 0.20) - full X range
    - y_min = -0.04 (was -0.045) - conservative right limit
    - y_step = 0.01 (was 0.001) - reasonable resolution
    
    Args:
        robot: Robot instance
        svg_file: Path to SVG file
        analysis: SVG analysis dict
        page_width_mm: Drawing width (mm)
        page_height_mm: Drawing height (mm)
        z_contact: Z-contact height (m)
        pitch: Pitch angle (radians)
        x_min, x_max, x_step: X search range
        y_min, y_max, y_step: Y search range
    
    Returns:
        dict: Best center position with reachability stats
    """
    print("\n" + "="*70)
    print("🔍 SCANNING WORKSPACE FOR OPTIMAL CENTER")
    print("="*70)
    
    actual_bounds = analysis['actual_bounds']
    
    # Calculate grid
    x_positions = []
    x_current = x_min
    while x_current <= x_max:
        x_positions.append(x_current)
        x_current += x_step
    
    y_positions = []
    y_current = y_min
    while y_current <= y_max:
        y_positions.append(y_current)
        y_current += y_step
    
    total_tests = len(x_positions) * len(y_positions)
    
    print(f"\nSearch grid:")
    print(f"  X range: [{x_min:.3f}, {x_max:.3f}] step {x_step:.3f} → {len(x_positions)} positions")
    print(f"  Y range: [{y_min:.3f}, {y_max:.3f}] step {y_step:.3f} → {len(y_positions)} positions")
    print(f"  Total tests: {total_tests}")
    print(f"\nDrawing size: {page_width_mm:.1f}mm × {page_height_mm:.1f}mm")
    print(f"Z-contact: {z_contact:.3f}m")
    
    response = input("\nStart scan? (yes/no): ")
    if response.lower() not in ['yes', 'y']:
        print("Scan cancelled")
        return None
    
    print("\n" + "="*70)
    print("SCANNING IN PROGRESS...")
    print("="*70)
    
    best_center = None
    best_fraction = 0.0
    best_reachable = 0
    best_total = 0
    
    results = []
    test_num = 0
    start_time = time.time()
    
    for center_x in x_positions:
        for center_y in y_positions:
            test_num += 1
            
            # Load SVG points for this center position
            points = load_and_scale_svg_from_center(
                svg_file, actual_bounds, page_width_mm, page_height_mm,
                center_x, center_y
            )
            
            if len(points) == 0:
                continue
            
            # Test reachability
            reachable, total, fraction = test_reachability_fast(
                robot, points, z_contact, pitch
            )
            
            results.append({
                'center_x': center_x,
                'center_y': center_y,
                'reachable': reachable,
                'total': total,
                'fraction': fraction
            })
            
            # Update best
            if fraction > best_fraction:
                best_fraction = fraction
                best_center = (center_x, center_y)
                best_reachable = reachable
                best_total = total
            
            # Progress update every 10 tests
            if test_num % 10 == 0 or test_num == total_tests:
                elapsed = time.time() - start_time
                eta = (elapsed / test_num) * (total_tests - test_num) if test_num > 0 else 0
                print(f"  Progress: {test_num}/{total_tests} ({100*test_num/total_tests:.0f}%) | "
                      f"Best so far: {best_fraction*100:.1f}% at ({best_center[0]:.3f}, {best_center[1]:.3f}) | "
                      f"ETA: {eta:.0f}s")
    
    print("\n" + "="*70)
    print("✓ SCAN COMPLETE")
    print("="*70)
    
    # Sort results by fraction
    results.sort(key=lambda r: r['fraction'], reverse=True)
    
    # Show top 5 results
    print("\n🏆 Top 5 Center Positions:")
    for i, result in enumerate(results[:5], 1):
        print(f"  {i}. Center: ({result['center_x']:.3f}, {result['center_y']:.3f}) | "
              f"Reachable: {result['reachable']}/{result['total']} ({result['fraction']*100:.1f}%)")
    
    # Show worst 3 for context
    if len(results) > 5:
        print(f"\n📉 Worst 3 Center Positions (for context):")
        for i, result in enumerate(results[-3:], 1):
            print(f"  {i}. Center: ({result['center_x']:.3f}, {result['center_y']:.3f}) | "
                  f"Reachable: {result['reachable']}/{result['total']} ({result['fraction']*100:.1f}%)")
    
    if best_center:
        print("\n" + "="*70)
        print("🎯 OPTIMAL CENTER FOUND")
        print("="*70)
        print(f"\nBest center position:")
        print(f"  X: {best_center[0]:.3f} m ({best_center[0]*1000:.1f} mm)")
        print(f"  Y: {best_center[1]:.3f} m ({best_center[1]*1000:.1f} mm)")
        print(f"  Reachability: {best_reachable}/{best_total} ({best_fraction*100:.1f}%)")
        
        # Show what Y range this creates for the drawing
        page_height_m = page_height_mm * 0.001
        y_bottom = best_center[1] - page_height_m / 2
        y_top = best_center[1] + page_height_m / 2
        print(f"\nDrawing Y extent:")
        print(f"  Bottom: {y_bottom:.3f} m ({y_bottom*1000:.1f} mm)")
        print(f"  Top: {y_top:.3f} m ({y_top*1000:.1f} mm)")
        
        if best_fraction >= 0.90:
            print(f"\n  ✅ Excellent! >90% reachable")
        elif best_fraction >= 0.70:
            print(f"\n  ⚠️  Good, but <90% reachable")
            print(f"  Consider making drawing smaller")
        else:
            print(f"\n  ❌ Poor reachability (<70%)")
            print(f"  Drawing is too large for this workspace")
            print(f"  Try reducing size to {page_width_mm*0.7:.0f}mm × {page_height_mm*0.7:.0f}mm")
        
        return {
            'center_x': best_center[0],
            'center_y': best_center[1],
            'reachable': best_reachable,
            'total': best_total,
            'fraction': best_fraction,
            'all_results': results
        }
    else:
        print("\n❌ No suitable center position found!")
        return None


def visualize_scan_results(results_data, svg_file):
    """
    Create a heatmap visualization of the scan results.
    """
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("\n⚠️  matplotlib not available, skipping visualization")
        return
    
    if not results_data or 'all_results' not in results_data:
        return
    
    results = results_data['all_results']
    
    # Extract data
    x_vals = [r['center_x'] for r in results]
    y_vals = [r['center_y'] for r in results]
    fractions = [r['fraction'] for r in results]
    
    # Create grid
    x_unique = sorted(set(x_vals))
    y_unique = sorted(set(y_vals))
    
    grid = np.zeros((len(y_unique), len(x_unique)))
    
    for result in results:
        i = y_unique.index(result['center_y'])
        j = x_unique.index(result['center_x'])
        grid[i, j] = result['fraction']
    
    # Plot
    fig, ax = plt.subplots(figsize=(12, 8))
    
    im = ax.imshow(grid, origin='lower', aspect='auto', cmap='RdYlGn',
                   extent=[min(x_unique), max(x_unique), min(y_unique), max(y_unique)])
    
    ax.set_xlabel('Center X (meters)', fontsize=12)
    ax.set_ylabel('Center Y (meters)', fontsize=12)
    ax.set_title('Reachability Heatmap\n(Green = High Reachability, Red = Low)', 
                 fontsize=14, fontweight='bold')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Reachability Fraction', fontsize=12)
    
    # Mark best position
    best_x = results_data['center_x']
    best_y = results_data['center_y']
    ax.plot(best_x, best_y, 'b*', markersize=20, markeredgecolor='white', 
            markeredgewidth=2, label=f'Best: ({best_x:.3f}, {best_y:.3f})')
    
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # Save
    output_file = 'workspace_scan_heatmap.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Heatmap saved to: {output_file}")
    
    plt.close()


def main():
    if len(sys.argv) < 2:
        print("Usage: python find_optimal_center_fixed.py <svg_file> [page_width_mm] [page_height_mm]")
        print("\nExample:")
        print("  python find_optimal_center_fixed.py butterfly.svg")
        print("  python find_optimal_center_fixed.py butterfly.svg 70 70")
        sys.exit(1)
    
    svg_file = sys.argv[1]
    
    # Get drawing size
    page_width_mm = 70.0
    page_height_mm = 70.0
    
    if len(sys.argv) > 2:
        page_width_mm = float(sys.argv[2])
    if len(sys.argv) > 3:
        page_height_mm = float(sys.argv[3])
    
    print("\n" + "="*70)
    print("OPTIMAL CENTER FINDER (CORRECTED)")
    print("="*70)
    print(f"\nSVG file: {svg_file}")
    print(f"Drawing size: {page_width_mm:.1f}mm × {page_height_mm:.1f}mm")
    
    # Analyze SVG
    min_x, min_y, max_x, max_y = get_path_bounds(svg_file)
    if min_x is None:
        print("❌ No paths found in SVG")
        sys.exit(1)
    
    analysis = {
        'actual_bounds': (min_x, min_y, max_x, max_y)
    }
    
    # Get parameters
    z_contact_input = input("\nEnter z_contact (meters, or Enter for -0.045): ").strip()
    z_contact = float(z_contact_input) if z_contact_input else -0.045
    
    pitch_input = input("Enter pitch (radians, or Enter for 0.0): ").strip()
    pitch = float(pitch_input) if pitch_input else 0.0
    
    # Scan parameters
    print("\n" + "="*70)
    print("SCAN PARAMETERS")
    print("="*70)
    print("\nDefault search area (CORRECTED):")
    print("  X: [0.17, 0.25] step 0.01 (9 positions)")
    print("  Y: [-0.04, 0.09] step 0.01 (14 positions) ← Less negative Y")
    print("  Total: 126 tests (~30-60 seconds)")
    
    use_default = input("\nUse default search area? (yes/no): ")
    
    if use_default.lower() in ['yes', 'y']:
        x_min, x_max, x_step = 0.17, 0.25, 0.01
        y_min, y_max, y_step = -0.04, 0.09, 0.01
    else:
        x_min = float(input("X min (m): "))
        x_max = float(input("X max (m): "))
        x_step = float(input("X step (m): "))
        y_min = float(input("Y min (m): "))
        y_max = float(input("Y max (m): "))
        y_step = float(input("Y step (m): "))
    
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
    
    # Run scan
    results = scan_workspace_grid(
        robot, svg_file, analysis,
        page_width_mm, page_height_mm,
        z_contact, pitch,
        x_min, x_max, x_step,
        y_min, y_max, y_step
    )
    
    if results:
        # Visualize
        visualize_scan_results(results, svg_file)
        
        # Ask to save
        print("\n" + "="*70)
        print("SAVE CALIBRATION")
        print("="*70)
        response = input("\nSave this as workspace calibration? (yes/no): ")
        
        if response.lower() in ['yes', 'y']:
            # Try to save with dimensions, fall back if old version
            try:
                save_calibration(
                    results['center_x'],
                    results['center_y'],
                    z_contact=z_contact,
                    page_width_mm=page_width_mm,
                    page_height_mm=page_height_mm
                )
            except TypeError:
                # Old version of workspace_calibration.py
                print("⚠️  Using older workspace_calibration.py (dimensions not saved)")
                save_calibration(
                    results['center_x'],
                    results['center_y'],
                    z_contact
                )
            print("\n✓ Calibration saved!")
            print(f"  You can now use auto_scale_svg_fixed.py with this center")
            print(f"  Make sure to use the same dimensions: {page_width_mm:.0f}mm × {page_height_mm:.0f}mm")
    else:
        print("\n❌ Scan failed - no suitable position found")
        print("\nSuggestions:")
        print("  1. Make drawing smaller")
        print("  2. Try different z_contact value")
        print("  3. Check robot workspace limits")


if __name__ == "__main__":
    main()
