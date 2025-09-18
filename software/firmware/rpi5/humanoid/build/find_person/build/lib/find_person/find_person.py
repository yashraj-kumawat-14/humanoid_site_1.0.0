import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FindPerson(Node):
    def __init__(self):
        super().__init__("node_find_person")

        # Subscription to receive requests for a person to find (face image path)
        self.create_subscription(
            String,
            "find_person_topic",
            self.request_callback,
            10
        )

        # Publisher to send face path to FaceRecognizer node
        self.face_ref_pub = self.create_publisher(String, "face_reference", 10)

        # Subscription to get detection results from FaceRecognizer node
        self.create_subscription(
            String,
            "face_detection",
            self.result_callback,
            10
        )

        # Publisher to output final result
        self.result_pub = self.create_publisher(String, "person_found_topic", 10)

        self.get_logger().info("FindPerson node initialized")

    def request_callback(self, msg):
        """Receive the path to the face image to find and send it to FaceRecognizer"""
        path = msg.data
        self.get_logger().info(f"Received request to find: {path}")

        ref_msg = String()
        ref_msg.data = path
        self.face_ref_pub.publish(ref_msg)
        self.get_logger().info(f"Published reference face path to FaceRecognizer: {path}")

    def result_callback(self, msg):
        """Receive the detection result from FaceRecognizer and forward it"""
        status = msg.data
        self.get_logger().info(f"FaceRecognizer result: {status}")

        out_msg = String()
        out_msg.data = status
        self.result_pub.publish(out_msg)
        self.get_logger().info(f"Published final result to person_found_topic: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = FindPerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

