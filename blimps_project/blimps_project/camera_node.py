import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        # Create a ROS2 publisher for the camera feed
        self.publisher_ = self.create_publisher(Image, '/blimp/camera', 10)

        # Open webcam (0 = default /dev/video0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera. Check /dev/video0 access.")

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_frame)  # Publish at ~10 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert frame to ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_image)

            # Display video feed using OpenCV
            cv2.imshow("Blimp Camera Feed", frame)
            cv2.waitKey(1)  # Allows the window to refresh

            self.get_logger().info("Publishing webcam frame")
        else:
            self.get_logger().warning("Failed to capture frame.")

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()  # Close OpenCV window
        super().destroy_node()


def main():
    rclpy.init()
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

