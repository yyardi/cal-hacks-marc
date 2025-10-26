#!/usr/bin/env python

"""
SVG to Robot Coordinate Transformation

This module converts SVG vector paths into robot drawing commands.
Pipeline: SVG pixels → Page mm → Robot meters → Joint angles
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from xml.dom import minidom
from svgpathtools import parse_path, Path, Line, CubicBezier, QuadraticBezier, Arc

from robot_movement import RobotMovementController, DrawingPath


class SVGToRobotTransformer:
    """
    Transforms SVG paths to robot coordinates with homography support.
    """
    
    def __init__(self, 
                 svg_width_px: float,
                 svg_height_px: float,
                 page_width_mm: float = 210.0,  # A4 width
                 page_height_mm: float = 297.0,  # A4 height
                 homography_matrix: Optional[np.ndarray] = None):
        """
        Initialize the SVG to robot transformer.
        
        Args:
            svg_width_px: Width of SVG viewBox in pixels
            svg_height_px: Height of SVG viewBox in pixels
            page_width_mm: Physical page width in millimeters
            page_height_mm: Physical page height in millimeters
            homography_matrix: Optional 3x3 homography from camera calibration
        """
        self.svg_width_px = svg_width_px
        self.svg_height_px = svg_height_px
        self.page_width_mm = page_width_mm
        self.page_height_mm = page_height_mm
        self.homography_matrix = homography_matrix
        
        # Robot coordinate system parameters
        # These should match your PaintBotDrawer settings
        self.page_origin_x_robot = 0.15  # Robot X coord of page origin (meters)
        self.page_origin_y_robot = -0.10  # Robot Y coord of page origin (meters)
        self.page_rotation = 0.0  # Page rotation in radians
        self.mm_to_meters = 0.001
        
    def svg_to_page(self, x_svg: float, y_svg: float) -> Tuple[float, float]:
        """
        Convert SVG pixel coordinates to page millimeters.
        
        Args:
            x_svg: X coordinate in SVG pixels
            y_svg: Y coordinate in SVG pixels
            
        Returns:
            (x_mm, y_mm) on page
        """
        # Apply homography if available (from camera calibration)
        if self.homography_matrix is not None:
            point = np.array([x_svg, y_svg, 1.0])
            transformed = self.homography_matrix @ point
            x_mm = transformed[0] / transformed[2]
            y_mm = transformed[1] / transformed[2]
        else:
            # Simple linear scaling (no camera calibration)
            x_mm = (x_svg / self.svg_width_px) * self.page_width_mm
            y_mm = (y_svg / self.svg_height_px) * self.page_height_mm
        
        return x_mm, y_mm
    
    def page_to_robot(self, x_mm: float, y_mm: float) -> Tuple[float, float]:
        """
        Convert page millimeters to robot base coordinates (meters).
        
        Args:
            x_mm: X coordinate on page in mm
            y_mm: Y coordinate on page in mm
            
        Returns:
            (x_robot, y_robot) in meters
        """
        # Convert mm to meters
        x_m = x_mm * self.mm_to_meters
        y_m = y_mm * self.mm_to_meters
        
        # Apply rotation if page is rotated relative to robot
        if self.page_rotation != 0:
            cos_r = math.cos(self.page_rotation)
            sin_r = math.sin(self.page_rotation)
            x_rot = x_m * cos_r - y_m * sin_r
            y_rot = x_m * sin_r + y_m * cos_r
        else:
            x_rot, y_rot = x_m, y_m
        
        # Add origin offset
        x_robot = self.page_origin_x_robot + x_rot
        y_robot = self.page_origin_y_robot + y_rot
        
        return x_robot, y_robot
    
    def svg_to_robot(self, x_svg: float, y_svg: float) -> Tuple[float, float]:
        """
        Direct conversion from SVG pixels to robot coordinates.
        
        Args:
            x_svg: X coordinate in SVG pixels
            y_svg: Y coordinate in SVG pixels
            
        Returns:
            (x_robot, y_robot) in meters
        """
        x_mm, y_mm = self.svg_to_page(x_svg, y_svg)
        return self.page_to_robot(x_mm, y_mm)
    
    def check_reachable(self, x_robot: float, y_robot: float,
                       x_min: float = 0.15, x_max: float = 0.25,
                       y_min: float = -0.10, y_max: float = 0.10) -> bool:
        """
        Check if a robot coordinate is within the safe workspace.
        
        Args:
            x_robot, y_robot: Robot coordinates in meters
            x_min, x_max, y_min, y_max: Workspace bounds
            
        Returns:
            True if reachable, False otherwise
        """
        return (x_min <= x_robot <= x_max and 
                y_min <= y_robot <= y_max)


class SVGPathLoader:
    """
    Loads and processes SVG paths for robot drawing.
    """
    
    def __init__(self, svg_file: str):
        """
        Initialize SVG loader.
        
        Args:
            svg_file: Path to SVG file
        """
        self.svg_file = svg_file
        self.doc = minidom.parse(svg_file)
        
        # Get SVG dimensions from viewBox or width/height
        svg_elem = self.doc.getElementsByTagName('svg')[0]
        self.width, self.height = self._get_svg_dimensions(svg_elem)
        
    def _get_svg_dimensions(self, svg_elem) -> Tuple[float, float]:
        """Extract SVG dimensions from viewBox or width/height attributes."""
        # Try viewBox first
        viewbox = svg_elem.getAttribute('viewBox')
        if viewbox:
            parts = viewbox.split()
            if len(parts) == 4:
                return float(parts[2]), float(parts[3])
        
        # Fall back to width/height
        width = svg_elem.getAttribute('width')
        height = svg_elem.getAttribute('height')
        
        if width and height:
            # Remove units if present
            width = float(width.replace('px', '').replace('pt', ''))
            height = float(height.replace('px', '').replace('pt', ''))
            return width, height
        
        # Default fallback
        return 500.0, 500.0
    
    def load_paths(self, num_points_per_segment: int = 10) -> List[List[Tuple[float, float]]]:
        """
        Load all paths from SVG and convert to point lists.
        
        Args:
            num_points_per_segment: Number of points to sample per path segment
            
        Returns:
            List of paths, where each path is a list of (x, y) tuples in SVG pixels
        """
        all_paths = []
        
        for path_elem in self.doc.getElementsByTagName('path'):
            d = path_elem.getAttribute('d')
            if not d:
                continue
            
            try:
                svg_path = parse_path(d)
                points = self._discretize_path(svg_path, num_points_per_segment)
                if points:
                    all_paths.append(points)
            except Exception as e:
                print(f"Warning: Failed to parse path: {e}")
                continue
        
        return all_paths
    
    def _discretize_path(self, svg_path: Path, num_points: int) -> List[Tuple[float, float]]:
        """
        Convert an SVG path to a list of discrete points.
        
        Args:
            svg_path: svgpathtools Path object
            num_points: Points to sample per segment
            
        Returns:
            List of (x, y) tuples
        """
        points = []
        
        for segment in svg_path:
            # Sample points along the segment
            for i in range(num_points):
                t = i / (num_points - 1) if num_points > 1 else 0
                point = segment.point(t)
                points.append((point.real, point.imag))
        
        # Add final point
        if svg_path:
            final = svg_path.point(1.0)
            points.append((final.real, final.imag))
        
        return points
    
    def load_paths_by_color(self) -> dict:
        """
        Load paths grouped by stroke color (for multi-color drawing).
        
        Returns:
            Dictionary mapping color strings to lists of paths
        """
        paths_by_color = {}
        
        for path_elem in self.doc.getElementsByTagName('path'):
            d = path_elem.getAttribute('d')
            if not d:
                continue
            
            # Get stroke color
            stroke = path_elem.getAttribute('stroke') or 'black'
            style = path_elem.getAttribute('style')
            
            # Parse stroke from style if present
            if style and 'stroke:' in style:
                for part in style.split(';'):
                    if 'stroke:' in part:
                        stroke = part.split(':')[1].strip()
            
            try:
                svg_path = parse_path(d)
                points = self._discretize_path(svg_path, num_points=10)
                
                if points:
                    if stroke not in paths_by_color:
                        paths_by_color[stroke] = []
                    paths_by_color[stroke].append(points)
            except Exception as e:
                print(f"Warning: Failed to parse path: {e}")
        
        return paths_by_color


def svg_to_drawing_paths(svg_file: str,
                         transformer: SVGToRobotTransformer,
                         z_contact: float = 0.0,
                         z_safe: float = 0.05) -> List[DrawingPath]:
    """
    Load SVG and convert to DrawingPath objects ready for robot execution.
    
    Args:
        svg_file: Path to SVG file
        transformer: SVGToRobotTransformer instance
        z_contact: Z height when drawing
        z_safe: Z height when traveling
        
    Returns:
        List of DrawingPath objects
    """
    # Load SVG
    loader = SVGPathLoader(svg_file)
    svg_paths = loader.load_paths(num_points_per_segment=10)
    
    print(f"Loaded {len(svg_paths)} paths from {svg_file}")
    print(f"SVG dimensions: {loader.width}px × {loader.height}px")
    
    # Convert to robot paths
    drawing_paths = []
    skipped_count = 0
    
    for i, svg_path_points in enumerate(svg_paths):
        path = DrawingPath(z_contact=z_contact, z_safe=z_safe)
        
        # Check if path is within workspace
        all_reachable = True
        robot_points = []
        
        for x_svg, y_svg in svg_path_points:
            x_robot, y_robot = transformer.svg_to_robot(x_svg, y_svg)
            robot_points.append((x_robot, y_robot))
            
            if not transformer.check_reachable(x_robot, y_robot):
                all_reachable = False
                print(f"Warning: Path {i+1} has unreachable points")
                break
        
        if not all_reachable:
            skipped_count += 1
            continue
        
        # Build drawing path
        # Move to start (pen up)
        if robot_points:
            path.add_point(robot_points[0][0], robot_points[0][1], pen_down=False)
            
            # Draw path (pen down)
            for x_robot, y_robot in robot_points:
                path.add_point(x_robot, y_robot, pen_down=True)
            
            # Lift pen at end
            path.add_point(robot_points[-1][0], robot_points[-1][1], pen_down=False)
            
            drawing_paths.append(path)
    
    if skipped_count > 0:
        print(f"Skipped {skipped_count} paths (outside workspace)")
    
    print(f"Created {len(drawing_paths)} drawing paths")
    
    return drawing_paths


# Example usage
if __name__ == "__main__":
    import sys
    from robot_movement import SO100Follower, SO100FollowerConfig
    
    if len(sys.argv) < 2:
        print("Usage: python svg_to_robot.py <svg_file>")
        print("\nExample:")
        print("  python svg_to_robot.py butterfly.svg")
        sys.exit(1)
    
    svg_file = sys.argv[1]
    
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
    
    # Load SVG and get dimensions
    loader = SVGPathLoader(svg_file)
    print(f"\nSVG Info:")
    print(f"  File: {svg_file}")
    print(f"  Dimensions: {loader.width}px × {loader.height}px")
    
    # Create transformer
    # Option 1: Simple scaling (no camera)
    transformer = SVGToRobotTransformer(
        svg_width_px=loader.width,
        svg_height_px=loader.height,
        page_width_mm=200.0,   # Scale to 200mm width
        page_height_mm=200.0,  # Scale to 200mm height
    )
    
    # Option 2: With homography (from AprilTag calibration)
    # H = np.array([...])  # Your homography matrix
    # transformer = SVGToRobotTransformer(
    #     svg_width_px=loader.width,
    #     svg_height_px=loader.height,
    #     homography_matrix=H
    # )
    
    # Update transformer with your calibrated values
    transformer.page_origin_x_robot = 0.15  # Your calibrated origin
    transformer.page_origin_y_robot = -0.10
    
    # Convert SVG to drawing paths
    drawing_paths = svg_to_drawing_paths(
        svg_file,
        transformer,
        z_contact=0.0,  # Your calibrated z_contact
        z_safe=0.05
    )
    
    if not drawing_paths:
        print("\nNo valid paths found or all paths outside workspace!")
        sys.exit(1)
    
    # Preview coordinate transformation for first point
    if drawing_paths[0].waypoints:
        first_point = drawing_paths[0].waypoints[0]
        print(f"\nFirst point transformation:")
        print(f"  Robot coordinates: ({first_point[0]:.3f}, {first_point[1]:.3f}, {first_point[2]:.3f})")
    
    # Ask user to confirm
    print(f"\nReady to draw {len(drawing_paths)} paths")
    response = input("Continue? (y/n): ")
    
    if response.lower() == 'y':
        # Move to safe position
        controller.move_to_safe_position(duration=3.0)
        
        # Execute all paths
        for i, path in enumerate(drawing_paths):
            print(f"\nDrawing path {i+1}/{len(drawing_paths)}")
            success = path.execute(controller, move_duration=0.3)
            if not success:
                print(f"Failed to complete path {i+1}")
                break
        
        # Return to safe position
        controller.move_to_safe_position(duration=3.0)
        print("\nDrawing complete!")
    else:
        print("Drawing cancelled")
