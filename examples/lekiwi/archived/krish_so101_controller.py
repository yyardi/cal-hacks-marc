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

from paintbot_examples import PaintBotDrawer

# Create drawer (handles coordinate transforms)
drawer = PaintBotDrawer(controller)

# Draw paths in page coordinates (mm)
paths = [
    [(50, 50), (100, 50)],  # Horizontal line
    [(100, 50), (100, 100)],  # Vertical line
]

drawer.draw_multiple_paths(paths)