import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SpeakTester(Node):
    def __init__(self):
        super().__init__("node_speak_tester")

        # Publisher -> trigger speaking
        self.pub_tts = self.create_publisher(String, "piper_tts_text", 10)

        # Subscriber -> check finished event
        self.create_subscription(String, "piper_tts_done", self.done_callback, 10)

        # Start test after short delay
        self.timer = self.create_timer(2.0, self.start_test)
        self.test_started = False

    def start_test(self):
        if self.test_started:
            return
        self.test_started = True

        msg = String()
        msg.data = "Hello, this is a test of the speaking system. I will notify when finished."
        self.pub_tts.publish(msg)
        self.get_logger().info("📢 Sent test message to Speak node")

    def done_callback(self, msg: String):
        if msg.data.strip().lower() == "finished":
            self.get_logger().info("✅ Speak node reported finished!")
            # optionally shut down test node
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SpeakTester()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

