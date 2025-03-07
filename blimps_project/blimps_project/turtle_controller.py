import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
import time
import math

# PID Constants
Kp_linear = 1.5  # Proportional gain for linear speed
Ki_linear = 0.01  # Integral gain for linear speed
Kd_linear = 0.02  # Derivative gain for linear speed

Kp_angular = 2.5  # Proportional gain for angular speed
Ki_angular = 0.02  # Integral gain for angular speed
Kd_angular = 0.1  # Derivative gain for angular speed

GOAL_TOLERANCE = 0.3  # Turtle is considered "at target" within this distance
HEADING_TOLERANCE = 0.1  # Allowable angle difference before moving forward
MAX_LINEAR_SPEED = 2.0  # Maximum linear speed
MIN_LINEAR_SPEED = 0.2  # Minimum linear speed

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # Subscribe to blob detection goal
        self.subscription = self.create_subscription(
            String, '/blimp/blobs', self.set_target, 10)

        # Subscribe to turtle pose updates
        self.pose_subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.update_pose, 10)

        # Publisher to control turtle movement
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Publisher to notify blob detector when the turtle reaches the goal
        self.reached_publisher_ = self.create_publisher(String, '/turtle_reached', 10)

        self.turtle_x = 5.5  # Default turtlesim start position
        self.turtle_y = 5.5
        self.turtle_theta = 0.0  # Orientation of the turtle
        self.target_x = None
        self.target_y = None
        self.reached_goal = False  # Flag to track if the goal has been reached

        self.previous_linear_error = 0
        self.previous_angular_error = 0
        self.integral_linear = 0
        self.integral_angular = 0
        self.last_time = time.time()

        self.get_logger().info("Turtle Controller Waiting for Goal")

    def update_pose(self, msg):
        """Updates the turtle's current position and heading, checks if the goal is reached."""
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta  # Heading in radians

        # If the turtle has a target, check if it has reached the goal
        if self.target_x is not None and self.target_y is not None:
            distance_to_goal = math.sqrt(
                (self.target_x - self.turtle_x) ** 2 + (self.target_y - self.turtle_y) ** 2
            )

            if distance_to_goal < GOAL_TOLERANCE and not self.reached_goal:
                self.get_logger().info("🎯 Turtle has reached the goal!")
                self.stop_turtle()
                self.reached_goal = True  # Prevent duplicate messages
                self.reached_publisher_.publish(String(data="reached"))  # Notify blob detector
                return

            # Move towards goal
            self.move_to_target(distance_to_goal)

    def set_target(self, msg):
        """Receives goal coordinates from the blob detector."""
        blobs = eval(msg.data)  # Convert string message to list
        if not blobs:
            return

        # Extract the first detected blob's X, Y position
        color, x, y, radius = blobs[0]

        # Convert blob's (X, Y) from image coordinates to turtlesim coordinates (scaled)
        self.target_x = ((x + 320) / 640) * 10  # Scale image width (0 to 640) to turtlesim (0 to 10)
        self.target_y = ((240 - y) / 480) * 10  # Scale image height (0 to 480) to turtlesim (0 to 10)

        self.reached_goal = False  # Reset flag when a new goal is set
        self.get_logger().info(f"📍 New Goal Set: ({self.target_x}, {self.target_y})")

    def move_to_target(self, distance_to_goal):
        """Move turtle towards target using PID control with speed adjustment based on distance."""
        if self.target_x is None or self.target_y is None or self.reached_goal:
            return  # Do nothing if no goal or already reached goal

        # Compute heading angle to the target
        error_x = self.target_x - self.turtle_x
        error_y = self.target_y - self.turtle_y
        target_angle = math.atan2(error_y, error_x)
        angular_error = target_angle - self.turtle_theta

        # Normalize angular error to be within -pi to pi
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        # PID Control for angular movement
        self.integral_angular += angular_error
        derivative_angular = angular_error - self.previous_angular_error
        angular_speed = (Kp_angular * angular_error) + (Ki_angular * self.integral_angular) + (Kd_angular * derivative_angular)
        self.previous_angular_error = angular_error

        # Create velocity message
        twist = Twist()

        # If the turtle is not facing the goal, turn first
        if abs(angular_error) > HEADING_TOLERANCE:
            twist.angular.z = min(max(angular_speed, -2.0), 2.0)  # Limit angular speed
            twist.linear.x = 0.0  # Do not move forward until facing the goal
        else:
            # Adjust linear speed dynamically based on distance to goal
            speed_scaling_factor = distance_to_goal / 10  # Normalize based on max distance in turtlesim
            linear_speed = max(MIN_LINEAR_SPEED, min(MAX_LINEAR_SPEED, Kp_linear * distance_to_goal * speed_scaling_factor))

            twist.linear.x = linear_speed  # Move forward
            twist.angular.z = 0.0  # Stop turning once aligned

        self.publisher_.publish(twist)

    def stop_turtle(self):
        """Stop movement after reaching the target and prevent rotation."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0  # Ensure no further rotation
        self.publisher_.publish(twist)


def main():
    rclpy.init()
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_turtle()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
