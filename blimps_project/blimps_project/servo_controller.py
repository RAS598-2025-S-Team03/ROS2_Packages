import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pigpio

# Servo Configuration
RIGHT_SERVO_PIN = 18  # Right servo on GPIO 18 (Physical Pin 12)
LEFT_SERVO_PIN = 17   # Left servo on GPIO 17 (Physical Pin 11)
PWM_FREQUENCY = 50  # 50Hz standard servo frequency

# Servo pulse width values (Adjust if needed)
CENTER_PULSE = 1500  # Center position (1.5ms pulse width)
LEFT_TURN_RIGHT_SERVO = 1300  # Right servo angle for a left turn (slightly left)
LEFT_TURN_LEFT_SERVO = 1000  # Left servo angle for a left turn (full left)
RIGHT_TURN_RIGHT_SERVO = 2000  # Right servo angle for a right turn (full right)
RIGHT_TURN_LEFT_SERVO = 1700  # Left servo angle for a right turn (slightly right)

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Subscribe to the detected blobs
        self.subscription = self.create_subscription(
            String, '/blimp/blobs', self.process_blob_data, 10)

        # Publish servo movement status
        self.servo_publisher = self.create_publisher(String, '/blimp/servo_status', 10)

        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpio daemon! Run 'sudo pigpiod'.")
            exit(1)

        self.get_logger().info("Servo Controller Node Started")
        self.center_servos()

    def process_blob_data(self, msg):
        """Adjust both servo positions based on detected blob location and publish movement status."""
        blobs = eval(msg.data)  # Convert string message to list

        if not blobs:
            self.get_logger().info("No objects detected. Keeping servos centered.")
            self.center_servos()
            self.publish_servo_status("Centered: No Blob Detected")
            return

        # Take the first detected blob
        color, x, y, radius = blobs[0]

        # Determine servo movement based on X position
        if x > 0:
            self.get_logger().info("Blob detected on the right. Both servos turning right.")
            self.pi.set_servo_pulsewidth(RIGHT_SERVO_PIN, RIGHT_TURN_RIGHT_SERVO)  # Right servo full right
            self.pi.set_servo_pulsewidth(LEFT_SERVO_PIN, RIGHT_TURN_LEFT_SERVO)  # Left servo slightly right
            self.publish_servo_status("Right Turn: Both servos turning right")

        elif x < 0:
            self.get_logger().info("Blob detected on the left. Both servos turning left.")
            self.pi.set_servo_pulsewidth(RIGHT_SERVO_PIN, LEFT_TURN_RIGHT_SERVO)  # Right servo slightly left
            self.pi.set_servo_pulsewidth(LEFT_SERVO_PIN, LEFT_TURN_LEFT_SERVO)  # Left servo full left
            self.publish_servo_status("Left Turn: Both servos turning left")

        else:
            self.get_logger().info("Blob centered. Resetting servos.")
            self.center_servos()
            self.publish_servo_status("Centered: Blob Centered")

    def center_servos(self):
        """Move both servos to center position."""
        self.pi.set_servo_pulsewidth(RIGHT_SERVO_PIN, CENTER_PULSE)
        self.pi.set_servo_pulsewidth(LEFT_SERVO_PIN, CENTER_PULSE)

    def publish_servo_status(self, status):
        """Publish the current servo movement status."""
        msg = String()
        msg.data = status
        self.servo_publisher.publish(msg)
        self.get_logger().info(f"Published Servo Status: {status}")

    def destroy(self):
        """Cleanup on shutdown."""
        self.pi.set_servo_pulsewidth(RIGHT_SERVO_PIN, 0)  # Stop PWM signal
        self.pi.set_servo_pulsewidth(LEFT_SERVO_PIN, 0)  # Stop PWM signal
        self.pi.stop()
        self.get_logger().info("Servo Controller Node Shutting Down")


def main():
    rclpy.init()
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
