import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Filtering parameters
MIN_RADIUS = 20   # Ignore small circles
MIN_AREA = 500    # Ignore small blobs
COLOR_PURITY_THRESHOLD = 0.3  # % of pixels matching the dominant color inside the circle

class ColorBlobDetector(Node):
    def __init__(self):
        super().__init__('color_blob_detector')

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image, '/blimp/camera', self.process_frame, 10)

        # Subscribe to turtle position updates
        self.turtle_subscription = self.create_subscription(
            String, '/turtle_reached', self.reset_goal, 10)

        # Publish detected blob data
        self.publisher_ = self.create_publisher(String, '/blimp/blobs', 10)

        self.bridge = CvBridge()
        self.goal_sent = False  # Prevents continuous goal updates
        self.detected_blobs = []  # Store detected blobs for visualization
        self.get_logger().info("Improved Color Blob Detector with Sorted Goals Started")

    def process_frame(self, msg):
        """Detect circular objects, classify color, and transform coordinates into quadrant system."""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Get image dimensions
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2  # Set midpoint as (0,0)

        # Convert to grayscale and apply Gaussian blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        # Detect circles using an improved Hough Transform
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                                   param1=70, param2=40, minRadius=MIN_RADIUS, maxRadius=150)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.detected_blobs = []  # Clear old detected blobs

        if circles is not None:
            circles = np.uint16(np.around(circles))

            for circle in circles[0, :]:
                x, y, radius = circle
                if radius < MIN_RADIUS:
                    continue  # Ignore very small circles

                # Convert to quadrant coordinate system (center = (0,0))
                adj_x = x - center_x
                adj_y = center_y - y  # Invert Y to align with Cartesian coordinates

                # Create a circular mask to extract color
                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), radius, (255, 255, 255), thickness=-1)
                mean_hsv = cv2.mean(hsv, mask=mask)[:3]

                # Determine the dominant color inside the detected circle
                color = self.get_color_label(hsv, mask)
                if not color:
                    continue  # Skip if no valid color is found

                self.detected_blobs.append((color, adj_x, adj_y, radius, x, y))

        # Sort blobs first by size (largest first)
        self.detected_blobs.sort(key=lambda b: -b[3])

        # Publish the largest detected blob as the goal if no goal is currently active
        if self.detected_blobs and not self.goal_sent:
            largest_blob = self.detected_blobs[0]
            color, adj_x, adj_y, radius, x, y = largest_blob
            self.publisher_.publish(String(data=str([(color, adj_x, adj_y, radius)])))
            self.get_logger().info(f'Published Goal: {color} at ({adj_x}, {adj_y})')
            self.goal_sent = True  # Prevent further updates until turtle reaches goal

        # Draw all detected blobs on the frame
        for color, adj_x, adj_y, radius, x, y in self.detected_blobs:
            cv2.circle(frame, (x, y), radius, (255, 255, 0), 2)
            cv2.putText(frame, f"{color} ({adj_x}, {adj_y})", (x - 40, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw quadrant grid on the frame
        self.draw_grid(frame, center_x, center_y)

        # Show debug video
        cv2.imshow('Blob Detection with Sorted Goals', frame)
        cv2.waitKey(1)

    def reset_goal(self, msg):
        """Resets goal when turtle reaches its destination, allowing new blob selection."""
        self.get_logger().info("Turtle reached goal. Selecting next largest blob.")
        self.goal_sent = False  # Allow publishing the next largest blob

    def get_color_label(self, hsv, mask):
        """Classify color based on HSV values."""
        color_ranges = {
            'red': [(np.array([0, 100, 100]), np.array([10, 255, 255])),
                    (np.array([160, 100, 100]), np.array([180, 255, 255]))],
            'green': [(np.array([30, 80, 80]), np.array([90, 255, 255]))],
            'blue': [(np.array([100, 100, 100]), np.array([140, 255, 255]))]
        }

        max_purity = 0
        detected_color = None

        for color, ranges in color_ranges.items():
            for lower, upper in ranges:
                color_mask = cv2.inRange(hsv, lower, upper)
                color_pixels = cv2.bitwise_and(color_mask, mask)
                match_percentage = np.sum(color_pixels > 0) / (np.sum(mask > 0) + 1e-5)

                if match_percentage > max_purity and match_percentage > COLOR_PURITY_THRESHOLD:
                    max_purity = match_percentage
                    detected_color = color

        return detected_color

    def draw_grid(self, frame, center_x, center_y):
        """Draw quadrant grid lines on the frame."""
        height, width, _ = frame.shape

        # Vertical line
        cv2.line(frame, (center_x, 0), (center_x, height), (255, 255, 255), 2)

        # # Horizontal line
        # cv2.line(frame, (0, center_y), (width, center_y), (255, 255, 255), 2)

        # # Label quadrants
        # cv2.putText(frame, "Q2", (center_x - 50, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # cv2.putText(frame, "Q1", (center_x + 20, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # cv2.putText(frame, "Q3", (center_x - 50, center_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # cv2.putText(frame, "Q4", (center_x + 20, center_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    rclpy.init()
    node = ColorBlobDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
