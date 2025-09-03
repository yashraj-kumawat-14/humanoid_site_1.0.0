import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class Brain(Node):
	def __init__(self):
		super().__init__("node_brain")
		
		self.tts_publisher = self.create_publisher(String, "piper_tts_text", 10)
		self.timer = self.create_timer(3, self.callbackFunction)
		
	def callbackFunction(self):
		msg = String()
		msg.data = "hello"
		self.tts_publisher.publish(msg)
		self.get_logger().info(f"Publisher node is publishing")
		

def main(args=None):
	rclpy.init(args=args)
	brain_node = Brain()
	rclpy.spin(brain_node)
	brain_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == "__main__":
	main()
