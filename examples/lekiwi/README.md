# Robot Drawing Library for SO100

A clean, modular library for controlling the SO100 robot arm for drawing applications. Built on top of the inverse kinematics engine from `teleoperate.py`.

## Files

- **`robot_movement.py`**: Core library with IK solver and movement control
- **`paintbot_examples.py`**: Examples for drawing applications
- **`teleoperate.py`**: Original keyboard teleoperation code (reference)

## Quick Start

### 1. Basic Movement

```python
from robot_movement import RobotMovementController, SO100Follower, SO100FollowerConfig

# Initialize robot
robot_config = SO100FollowerConfig(
    port="/dev/ttyACM0",  # Adjust for your system
    id="marc",
    use_degrees=True  # IMPORTANT: Must be True
)
robot = SO100Follower(robot_config)
robot.connect()

# Create controller
controller = RobotMovementController(robot)

# Move to safe starting position
controller.move_to_safe_position(duration=3.0)

# Move to a specific point (x, y, z in meters, pitch in radians)
controller.move_to(x=0.20, y=0.05, z=0.15, pitch_rad=0.0, duration=1.0)
```

### 2. Drawing Paths

```python
from robot_movement import DrawingPath

# Create a path
path = DrawingPath(z_contact=0.0, z_safe=0.05)

# Add points (x, y in meters)
path.add_point(0.15, -0.05, pen_down=False)  # Move to start
path.add_point(0.15, -0.05, pen_down=True)   # Lower pen
path.add_point(0.25, 0.05, pen_down=True)    # Draw line
path.add_point(0.25, 0.05, pen_down=False)   # Lift pen

# Execute the path
path.execute(controller, move_duration=0.3)
```

### 3. Drawing from Page Coordinates

```python
from paintbot_examples import PaintBotDrawer

# Create drawer (handles coordinate transforms)
drawer = PaintBotDrawer(controller)

# Draw paths in page coordinates (mm)
paths = [
    [(50, 50), (100, 50)],  # Horizontal line
    [(100, 50), (100, 100)],  # Vertical line
]

drawer.draw_multiple_paths(paths)
```

## Coordinate Systems

### Robot Coordinates (meters)
- **X**: Forward from robot base (positive = forward)
- **Y**: Left/right from robot's perspective (positive = left)
- **Z**: Up/down (positive = up, Z=0 at shoulder lift joint)
- **Pitch**: End effector angle in world frame (radians)

### Page Coordinates (millimeters)
- Standard for drawing applications
- Origin typically at bottom-left of page
- A4 paper: 210mm × 297mm

### Transformation
The `PaintBotDrawer` class handles conversion from page coordinates to robot coordinates:
1. Apply rotation (if page is rotated relative to robot)
2. Scale from mm to meters
3. Add origin offset

## Key Classes

### `RobotMovementController`
Main interface for robot control.

**Methods:**
- `move_to(x, y, z, pitch_rad, duration, wait)`: Move to a position
- `inverse_kinematics(x, y, z, pitch_rad)`: Calculate joint angles
- `set_wrist_roll(angle_deg)`: Set wrist roll angle
- `set_gripper(position)`: Set gripper position
- `move_to_safe_position()`: Move to safe starting pose
- `get_current_state()`: Read current joint angles

### `DrawingPath`
Represents a sequence of drawing waypoints.

**Methods:**
- `add_point(x, y, pen_down, pitch_rad)`: Add a waypoint
- `execute(controller, move_duration)`: Execute the path

### `PaintBotDrawer`
High-level interface for drawing applications.

**Methods:**
- `draw_svg_path(path_points)`: Draw a single SVG path
- `draw_multiple_paths(paths)`: Draw multiple paths in sequence
- `page_to_robot_coords(x_page, y_page)`: Convert page coords to robot coords

## Configuration

### Important Parameters

**Arm Dimensions** (in `RobotMovementController.__init__`):
- `l1 = 0.1159` m: Shoulder to elbow length
- `l2 = 0.1375` m: Elbow to wrist length
- `l3 = 0.17` m: Wrist to end effector length

**Drawing Heights** (in `DrawingPath.__init__` or `PaintBotDrawer.__init__`):
- `z_contact`: Z height when pen touches paper (usually ~0.0)
- `z_safe`: Z height for traveling (usually 0.05-0.10 m above contact)

**Page Transform** (in `PaintBotDrawer.__init__`):
- `page_origin_x`, `page_origin_y`: Robot coords of page origin
- `page_rotation`: Page rotation relative to robot (radians)
- `page_to_robot_scale`: Conversion factor (typically 0.001 for mm → m)

### Critical Settings

**Robot Configuration:**
```python
SO100FollowerConfig(
    port="/dev/ttyACM0",  # Your port
    id="marc",
    use_degrees=True  # ⚠️ MUST BE TRUE for IK to work!
)
```

The `use_degrees=True` setting is **critical**. Without it, the robot reports joint positions as interpolated scalars between -100 and 100, which breaks the inverse kinematics.

## Workspace Limits

The robot's reachable workspace is approximately:
- **X**: 0.10 to 0.30 m (forward from base)
- **Y**: -0.15 to 0.15 m (left/right)
- **Z**: -0.05 to 0.20 m (up/down from shoulder)

**Run the workspace test to find your exact limits:**
```bash
python paintbot_examples.py 1
```

## Examples

The `paintbot_examples.py` file includes several examples:

1. **Test Workspace Limits**: Probe the reachable space
2. **Draw Text**: Draw simple letter strokes
3. **Draw Shape at Scale**: Draw a star pattern on A4 paper
4. **Homography Integration**: Template for camera-based drawing

```bash
# Run an example
python paintbot_examples.py 1  # Workspace test
python paintbot_examples.py 2  # Draw letter 'A'
python paintbot_examples.py 3  # Draw star
```

## Integration with PaintBot Project

For your full PaintBot system (from the PDF), you'll need to add:

### 1. Camera Calibration
```python
import cv2
import numpy as np

# Detect AprilTags at page corners
# detector = apriltag.Detector()
# ...

# Compute homography
corners_image = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
corners_page = np.array([[0, 0], [210, 0], [210, 297], [0, 297]])
H, _ = cv2.findHomography(corners_image, corners_page)

# Transform points
def image_to_page(x_img, y_img):
    point = np.array([x_img, y_img, 1.0])
    transformed = H @ point
    return transformed[0]/transformed[2], transformed[1]/transformed[2]
```

### 2. SVG Path Loading
```python
from xml.dom import minidom

def load_svg_paths(svg_file):
    doc = minidom.parse(svg_file)
    paths = []
    for path in doc.getElementsByTagName('path'):
        d = path.getAttribute('d')
        # Parse SVG path commands into (x, y) points
        # Use a library like svgpathtools or svg.path
        points = parse_svg_path(d)
        paths.append(points)
    return paths
```

### 3. Multi-Color Support
```python
class MultiColorDrawer:
    def __init__(self, controller, pen_positions):
        self.controller = controller
        self.pen_positions = pen_positions  # Dict: color -> (x, y, z)
    
    def pick_pen(self, color):
        """Move to pen rack and pick up a pen."""
        pos = self.pen_positions[color]
        # Move to pen location
        # Grip pen
        # Return to drawing area
        pass
    
    def draw_with_colors(self, paths_by_color):
        """Draw paths grouped by color."""
        for color, paths in paths_by_color.items():
            self.pick_pen(color)
            for path in paths:
                self.draw_path(path)
            self.return_pen(color)
```

## Calibration Procedure

### 1. Safe Position
The safe position (defined in `move_to_safe_position`) should:
- Keep the robot stable (won't fall over)
- Be within workspace limits
- Allow easy transition to drawing area

Current safe position: `x=0.175, y=0.0, z=0.1, pitch=0.0`

### 2. Page Origin
To calibrate the page origin:
1. Place your drawing surface/paper
2. Mark the origin point (e.g., bottom-left corner)
3. Manually jog robot to touch that point with pen
4. Record the (x, y, z) position
5. Update `page_origin_x` and `page_origin_y` in `PaintBotDrawer`

### 3. Z Heights
To calibrate drawing heights:
1. Place pen in end effector
2. Move robot over drawing surface
3. Lower slowly until pen just touches: record as `z_contact`
4. Lift 5-10 cm: record as `z_safe`

## Troubleshooting

### IK Fails to Find Solution
- **Cause**: Target position is outside reachable workspace
- **Fix**: Run workspace test to find valid range; adjust target position

### Robot Drifts During Drawing
- **Cause**: Insufficient torque to overcome gravity
- **Fix**: Increase move duration; break paths into smaller segments; adjust trajectories to minimize gravity effects

### Calibration Drift
- **Cause**: Loose joints; calibration not accurate
- **Fix**: Re-run calibration; ensure robot is firmly mounted; check for mechanical issues

### Pen Doesn't Touch Paper
- **Cause**: `z_contact` too high
- **Fix**: Measure actual contact height; adjust `z_contact` parameter

## Performance Notes

From the original code comments:
- The SO100 arm's torque may not fully overcome gravity when extended
- Numerical IK solver can occasionally find incorrect solutions
- Desired positions can drift slightly beyond physical limits

**Recommendations:**
- Use shorter path segments for better accuracy
- Increase move duration for smoother motion
- Test workspace limits before large drawings
- Keep pen paths within well-tested regions

## Next Steps

1. **Run workspace test** to understand your robot's reach
2. **Calibrate page origin** for your drawing surface
3. **Test simple paths** (lines, rectangles) before complex art
4. **Integrate camera system** for AprilTag-based calibration
5. **Add SVG loading** for vector art input
6. **Implement multi-color** pen changing system

## License

Based on code developed by SIGRobotics UIUC.
Original teleoperate.py uses Apache License 2.0.
