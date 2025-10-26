#!/usr/bin/env python

"""
SVG Drawing Placement Visualizer

Shows where the drawing will be placed in the robot workspace
and which points are reachable.
"""

import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from xml.dom import minidom
from svgpathtools import parse_path


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


def calculate_centered_origin(page_width_mm, page_height_mm, 
                              workspace_x_min=0.15, workspace_x_max=0.25,
                              workspace_y_min=-0.10, workspace_y_max=0.10):
    """Calculate robot origin to center the drawing in the workspace."""
    page_width_m = page_width_mm * 0.001
    page_height_m = page_height_mm * 0.001
    
    workspace_center_x = (workspace_x_min + workspace_x_max) / 2
    workspace_center_y = (workspace_y_min + workspace_y_max) / 2
    
    drawing_center_offset_x = page_width_m / 2
    drawing_center_offset_y = page_height_m / 2
    
    robot_origin_x = workspace_center_x - drawing_center_offset_x
    robot_origin_y = workspace_center_y - drawing_center_offset_y
    
    return robot_origin_x, robot_origin_y


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


def test_ik_reachability_simple(x, y, z):
    """
    Simple heuristic for reachability without running full IK.
    This is an approximation based on typical arm geometry.
    
    NOTE: This is MORE LENIENT with z-height than actual IK,
    treating table contact as reachable.
    """
    # Distance from base
    r = math.sqrt(x**2 + y**2)
    
    # Typical SO100 arm reach constraints
    # These are rough estimates - actual IK is more complex
    min_reach = 0.05  # Minimum reach (too close to base)
    max_reach = 0.30  # Maximum reach (arm fully extended)
    
    # Height constraints - VERY LENIENT
    # Allow deep negative z (table contact) as long as x,y are reachable
    min_z = -0.10  # Allow table contact (was -0.15, now more realistic)
    max_z = 0.25   # Maximum height
    
    # Check basic geometric constraints
    if r < min_reach or r > max_reach:
        return False
    
    # Be lenient with z - only reject if VERY far out of range
    if z < -0.10 or z > max_z:
        return False
    
    # Main constraint: X position (left side is hard to reach)
    # This is the PRIMARY constraint that matters
    if x < 0.17:  # Slightly more lenient than 0.16
        return False
    
    # Y constraint (don't go too far to sides)
    if abs(y) > 0.12:  # Slightly more lenient
        return False
    
    return True


def visualize_drawing_placement(svg_file, page_width_mm, page_height_mm, 
                                z_contact=-0.025):
    """
    Create a visualization showing where the drawing will be placed
    in the robot workspace.
    """
    print("\n" + "="*70)
    print("DIAGNOSTIC VISUALIZATION")
    print("="*70)
    
    # Get SVG info
    min_x, min_y, max_x, max_y = get_path_bounds(svg_file)
    actual_bounds = (min_x, min_y, max_x, max_y)
    
    # Calculate centered origin using REACHABLE workspace
    robot_origin_x, robot_origin_y = calculate_centered_origin(
        page_width_mm, page_height_mm,
        workspace_x_min=0.17,  # REACHABLE area
        workspace_x_max=0.25,
        workspace_y_min=-0.09,
        workspace_y_max=0.09
    )
    
    # Load paths
    robot_paths = load_and_scale_svg(
        svg_file, actual_bounds, page_width_mm, page_height_mm,
        robot_origin_x, robot_origin_y
    )
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # ============= LEFT PLOT: TOP-DOWN VIEW =============
    ax1.set_title('Top-Down View: Drawing Placement in Workspace', fontsize=14, fontweight='bold')
    ax1.set_xlabel('X (meters) - Forward/Backward', fontsize=12)
    ax1.set_ylabel('Y (meters) - Left/Right', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # Draw geometric workspace boundary (light)
    workspace_rect_geom = patches.Rectangle(
        (0.15, -0.10), 0.10, 0.20,
        linewidth=2, edgecolor='lightblue', facecolor='lightblue', alpha=0.1,
        linestyle='--', label='Geometric Workspace'
    )
    ax1.add_patch(workspace_rect_geom)
    
    # Draw REACHABLE workspace boundary (darker, primary)
    workspace_rect_reach = patches.Rectangle(
        (0.17, -0.09), 0.08, 0.18,
        linewidth=3, edgecolor='blue', facecolor='lightblue', alpha=0.3,
        label='Reachable Workspace'
    )
    ax1.add_patch(workspace_rect_reach)
    
    # Draw workspace center (of reachable area)
    ax1.plot(0.21, 0.00, 'b+', markersize=15, markeredgewidth=2, 
             label='Reachable Center')
    
    # Draw drawing boundary
    page_width_m = page_width_mm * 0.001
    page_height_m = page_height_mm * 0.001
    drawing_rect = patches.Rectangle(
        (robot_origin_x, robot_origin_y), page_width_m, page_height_m,
        linewidth=2, edgecolor='green', facecolor='lightgreen', alpha=0.3,
        label=f'Drawing Area ({page_width_mm:.0f}×{page_height_mm:.0f}mm)'
    )
    ax1.add_patch(drawing_rect)
    
    # Draw origin point
    ax1.plot(robot_origin_x, robot_origin_y, 'go', markersize=10, 
             label='Drawing Origin')
    
    # Draw drawing center
    drawing_center_x = robot_origin_x + page_width_m / 2
    drawing_center_y = robot_origin_y + page_height_m / 2
    ax1.plot(drawing_center_x, drawing_center_y, 'g+', markersize=15, 
             markeredgewidth=2, label='Drawing Center')
    
    # Test reachability and plot points
    reachable_points = []
    unreachable_points = []
    
    for path in robot_paths:
        for x, y in path[::10]:  # Sample every 10th point
            if test_ik_reachability_simple(x, y, z_contact):
                reachable_points.append((x, y))
            else:
                unreachable_points.append((x, y))
    
    # Plot reachable points
    if reachable_points:
        rx, ry = zip(*reachable_points)
        ax1.scatter(rx, ry, c='green', s=10, alpha=0.6, label='Reachable Points')
    
    # Plot unreachable points
    if unreachable_points:
        ux, uy = zip(*unreachable_points)
        ax1.scatter(ux, uy, c='red', s=10, alpha=0.6, label='Unreachable Points')
    
    # Add annotations
    ax1.annotate(f'Origin:\n({robot_origin_x:.3f}, {robot_origin_y:.3f})',
                xy=(robot_origin_x, robot_origin_y),
                xytext=(robot_origin_x-0.03, robot_origin_y-0.03),
                fontsize=9, ha='right',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7),
                arrowprops=dict(arrowstyle='->', lw=1.5))
    
    ax1.legend(loc='upper left', fontsize=9)
    ax1.set_xlim(0.10, 0.30)
    ax1.set_ylim(-0.15, 0.15)
    
    # ============= RIGHT PLOT: STATISTICS =============
    ax2.axis('off')
    
    total_points = len(reachable_points) + len(unreachable_points)
    reachable_pct = 100 * len(reachable_points) / total_points if total_points > 0 else 0
    unreachable_pct = 100 * len(unreachable_points) / total_points if total_points > 0 else 0
    
    stats_text = f"""
PLACEMENT STATISTICS
{'='*50}

WORKSPACE:
  Geometric: X=[0.15, 0.25] Y=[-0.10, 0.10]
  REACHABLE: X=[0.17, 0.25] Y=[-0.09, 0.09] ⚠️
  Note: Arm struggles with x < 0.17
  Center of reachable: (0.21, 0.00)

DRAWING:
  Size: {page_width_mm:.1f} × {page_height_mm:.1f} mm
  Origin: ({robot_origin_x:.3f}, {robot_origin_y:.3f})
  Center: ({drawing_center_x:.3f}, {drawing_center_y:.3f})

REACHABILITY (sampled points):
  Total: {total_points}
  Reachable: {len(reachable_points)} ({reachable_pct:.1f}%)
  Unreachable: {len(unreachable_points)} ({unreachable_pct:.1f}%)

Z-HEIGHT:
  z_contact: {z_contact:.3f} m ({z_contact*1000:.1f} mm)
  Note: Low z is OK (table contact)

DIAGNOSIS:
"""
    
    if reachable_pct < 50:
        stats_text += """
  ❌ CRITICAL: <50% reachable
  
  LIKELY CAUSES:
  1. Drawing too far LEFT (x < 0.17)
  2. Drawing too large for reachable area
  3. Drawing extends beyond workspace
  
  SOLUTIONS:
  1. Reduce size to 50mm × 50mm
  2. Drawing should be x > 0.17
  3. Use auto_scale_svg_centered.py
"""
    elif reachable_pct < 90:
        stats_text += """
  ⚠️  WARNING: <90% reachable
  
  SUGGESTIONS:
  1. Reduce drawing size by 20-30%
  2. Ensure drawing x > 0.17
  3. Check edges don't exceed bounds
"""
    else:
        stats_text += """
  ✅ GOOD: >90% reachable
  
  Drawing should work well!
"""
    
    ax2.text(0.05, 0.95, stats_text, transform=ax2.transAxes,
            fontsize=11, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    
    # Save figure
    output_file = 'drawing_placement_diagnostic.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {output_file}")
    
    # Show figure
    plt.show()
    
    # Print detailed analysis
    print("\n" + "="*70)
    print("DETAILED ANALYSIS")
    print("="*70)
    
    print(f"\nDrawing placement:")
    print(f"  Origin: ({robot_origin_x:.3f}, {robot_origin_y:.3f})")
    print(f"  Size: {page_width_mm:.1f}mm × {page_height_mm:.1f}mm")
    print(f"  Extent: X=[{robot_origin_x:.3f}, {robot_origin_x + page_width_m:.3f}]")
    print(f"          Y=[{robot_origin_y:.3f}, {robot_origin_y + page_height_m:.3f}]")
    
    print(f"\nWorkspace boundaries:")
    print(f"  Geometric: X=[0.15, 0.25] Y=[-0.10, 0.10]")
    print(f"  REACHABLE: X=[0.17, 0.25] Y=[-0.09, 0.09] ⚠️")
    print(f"  Note: Arm cannot reliably reach x < 0.17")
    
    print(f"\nReachability:")
    print(f"  {len(reachable_points)}/{total_points} points reachable ({reachable_pct:.1f}%)")
    
    if unreachable_points:
        ux, uy = zip(*unreachable_points)
        print(f"\nUnreachable region:")
        print(f"  X range: [{min(ux):.3f}, {max(ux):.3f}]")
        print(f"  Y range: [{min(uy):.3f}, {max(uy):.3f}]")
        
        # Analyze why unreachable
        left_of_reachable = sum(1 for x, y in unreachable_points if x < 0.17)
        left_of_workspace = sum(1 for x, y in unreachable_points if x < 0.15)
        right_of_workspace = sum(1 for x, y in unreachable_points if x > 0.25)
        below_workspace = sum(1 for x, y in unreachable_points if y < -0.10)
        above_workspace = sum(1 for x, y in unreachable_points if y > 0.10)
        
        print(f"\nWhy unreachable:")
        if left_of_reachable > 0:
            print(f"  - {left_of_reachable} points in 'dead zone' (x < 0.17) ⚠️ MAIN ISSUE")
        if left_of_workspace > 0:
            print(f"    ({left_of_workspace} of those are x < 0.15)")
        if right_of_workspace > 0:
            print(f"  - {right_of_workspace} points RIGHT of workspace (x > 0.25)")
        if below_workspace > 0:
            print(f"  - {below_workspace} points BELOW workspace (y < -0.10)")
        if above_workspace > 0:
            print(f"  - {above_workspace} points ABOVE workspace (y > 0.10)")
    
    print("\n" + "="*70)
    
    return reachable_pct


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python visualize_drawing_placement.py <svg_file> [page_width_mm] [page_height_mm] [z_contact]")
        print("\nExamples:")
        print("  python visualize_drawing_placement.py butterfly.svg")
        print("  python visualize_drawing_placement.py butterfly.svg 70 70")
        print("  python visualize_drawing_placement.py butterfly.svg 70 70 -0.025")
        sys.exit(1)
    
    svg_file = sys.argv[1]
    
    # Default values
    page_width_mm = 70.0
    page_height_mm = 70.0
    z_contact = -0.025
    
    if len(sys.argv) > 2:
        page_width_mm = float(sys.argv[2])
    if len(sys.argv) > 3:
        page_height_mm = float(sys.argv[3])
    if len(sys.argv) > 4:
        z_contact = float(sys.argv[4])
    
    print(f"\nVisualizing: {svg_file}")
    print(f"Drawing size: {page_width_mm:.1f}mm × {page_height_mm:.1f}mm")
    print(f"Z contact: {z_contact:.3f}m")
    
    reachability = visualize_drawing_placement(
        svg_file, page_width_mm, page_height_mm, z_contact
    )
    
    print("\n" + "="*70)
    print("RECOMMENDATIONS")
    print("="*70)
    
    if reachability < 50:
        print("\n❌ CRITICAL: Drawing is mostly unreachable!")
        print("\n⚠️  Primary issue: Drawing is likely too far LEFT (x < 0.17)")
        print("\nTry these parameters:")
        print(f"  page_width_mm = {page_width_mm * 0.4:.1f}  (60% smaller)")
        print(f"  page_height_mm = {page_height_mm * 0.4:.1f}")
        print(f"  OR use auto_scale_svg_centered.py (auto-centers in reachable zone)")
    elif reachability < 90:
        print("\n⚠️  WARNING: Some unreachable points")
        print("\nTry these parameters:")
        print(f"  page_width_mm = {page_width_mm * 0.7:.1f}  (30% smaller)")
        print(f"  page_height_mm = {page_height_mm * 0.7:.1f}")
        print(f"  OR use auto_scale_svg_centered.py (auto-centers in reachable zone)")
    else:
        print("\n✅ Drawing should work well!")
        print(f"\nUse these parameters:")
        print(f"  page_width_mm = {page_width_mm:.1f}")
        print(f"  page_height_mm = {page_height_mm:.1f}")
        print(f"  z_contact = {z_contact:.3f}")
        print("\n  OR use auto_scale_svg_centered.py for automatic centering")


if __name__ == "__main__":
    main()
