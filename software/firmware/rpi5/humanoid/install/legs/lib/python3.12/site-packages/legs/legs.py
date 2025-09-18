import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class Legs(Node):
    def __init__(self):
        super().__init__("node_legs")

        # Subscribe to ROS2 topic
        self.create_subscription(String, "legs_topic", self.callback, 10)

        # === GPIO UART Serial to Arduino ===
        # Make sure serial login shell is disabled on RPi
        # /dev/serial0 maps to GPIO14 (TX) / GPIO15 (RX)
        # Only TX from Pi → RX of Arduino
        self.arduino = serial.Serial('/dev/serial0', 38400, timeout=1)
        time.sleep(2)  # Give Arduino time to reset
        self.get_logger().info("UART over GPIO initialized")

    def callback(self, msg):
        cmd = msg.data.lower()
        arduino_cmd = ""

        if cmd == "forward":
            arduino_cmd = 'F'
        elif cmd == "backward":
            arduino_cmd = 'B'
        elif cmd == "left":
            arduino_cmd = 'L'
        elif cmd == "right":
            arduino_cmd = 'R'
        elif cmd == "stop":
            arduino_cmd = 'S'

        if arduino_cmd:
            self.arduino.write(arduino_cmd.encode())
            self.get_logger().info(f"Sent to Arduino: {arduino_cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = Legs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.arduino.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

