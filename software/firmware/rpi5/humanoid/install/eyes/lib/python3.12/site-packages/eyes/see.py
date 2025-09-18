import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class See(Node):
    def __init__(self):
        super().__init__("node_see")

        # Publisher for images
        self.publisher_ = self.create_publisher(Image, "camera_topic", 10)

        # Timer to capture and publish images at 10 Hz
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # OpenCV camera capture (0 = default camera)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera")
            return

        # Convert OpenCV image (BGR) to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info("Published camera frame")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = See()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

