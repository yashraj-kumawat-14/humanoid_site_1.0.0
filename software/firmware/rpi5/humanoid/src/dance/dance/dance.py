import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Dance(Node):
    def __init__(self):
        super().__init__("node_dance")
        self.create_subscription(String, "dance_topic", self.callback, 10)
        
        # Publisher to legs node
        self.legs_pub = self.create_publisher(String, "legs_topic", 10)

    def callback(self, msg):
        if msg.data.lower() == "dance":
            self.get_logger().info("💃 Starting dance routine...")
            self.perform_dance()

    def perform_dance(self):
        # Simple dance sequence
        sequence = [
            ("forward", 0.3),
            ("backward", 0.3),
            ("forward", 0.3),
            ("backward", 0.3),
            ("left", 0.5),
            ("right", 0.5),
            ("left", 0.5),
            ("right", 0.5),
            ("stop", 0.2)
        ]
        
        for cmd, duration in sequence:
            msg = String()
            msg.data = cmd
            self.legs_pub.publish(msg)
            self.get_logger().info(f"Sent to legs: {cmd}")
            time.sleep(duration)

def main(args=None):
    rclpy.init(args=args)
    node = Dance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

