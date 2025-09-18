import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Namaste(Node):
	def __init__(self):
		super().__init__("node_namaste")
		self.create_subscription(String, "namaste_topic", self.callback, 10)
		
	def callback(self, msg):
		cmd = msg.data.lower()
		if(cmd=="namaste"):
			self.get_logger().info("doing namaste")
			
			
def main(args=None):
	rclpy.init(args=args)
	node = Namaste(Node)
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	
if __name__=="__main__":
	main()
	
			
