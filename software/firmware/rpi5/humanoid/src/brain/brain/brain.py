import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Brain(Node):
    def __init__(self, name="Vedanshi"):
        super().__init__("node_brain")
        self.speaking = False
        self.name = name

        # === Publishers ===
        self.pub_tts = self.create_publisher(String, "piper_tts_text", 10)
        self.pub_conversation = self.create_publisher(String, "conversation_prompt", 10)
        self.legs_pub = self.create_publisher(String, "legs_topic", 10)
        self.dance_pub = self.create_publisher(String, "dance_topic", 10)

        # === Subscribers ===
        self.create_subscription(String, "listen_with_vosk", self.listen_callback, 10)
        self.create_subscription(String, "piper_tts_done", self.speaking_done, 10)
        self.create_subscription(String, "conversation_response", self.response_callback, 10)
        self.create_subscription(String, "humanoid_action", self.action_callback, 10)

        # === Initial greeting ===
#        self.say(f"hello my name is {name}, how can i help you")
        #self.create_timer(2.0, lambda: self.say(f"hello my name is {self.name}, how can I help you")).cancel()  # cancels immediately after running once
        self.greet_timer = self.create_timer(2.0, self.greet_once)

    def greet_once(self):
        self.say(f"hello my name is {self.name}, how can I help you")
        self.greet_timer.cancel()  # stops after first run



    # ========== Methods ==========
    
    def speaking_done(self, msg):
        if(msg.data=="finished"):
            self.get_logger().info("finished speaking")
            self.speaking = False

    def say(self, text: str):
        """Publish text to TTS and mark as speaking."""
        if(self.speaking):
            self.get_logger().info("Currently speaking try later.")
            return
        msg = String()
        msg.data = text
        self.pub_tts.publish(msg)
        self.speaking = True
        self.get_logger().info(f"Speaking: {text}")

    def listen_callback(self, msg: String):
        """Handle speech-to-text input from Vosk."""
        user_text = msg.data.strip()
        if not user_text:
            return
            
        if(self.speaking):
            self.get_logger().info("Igonoring heard text, currently speaking.")
            return
            
        self.get_logger().info(f"👂 Heard: {user_text}")
        #self.say(user_text)

        # Forward to conversation node
        prompt_msg = String()
        prompt_msg.data = user_text
        self.pub_conversation.publish(prompt_msg)
        self.get_logger().info("➡️ Sent prompt to conversation node")

    def response_callback(self, msg: String):
        """Handle text response from conversation node."""
        response_text = msg.data.strip()
        if response_text:
            self.say(response_text)

    def action_callback(self, msg: String):
        """Handle actions published by the conversation node."""
        action_name = msg.data.strip()
        if(action_name=="move_forward"):
            self.say("ok")
            msg = String()
            msg.data = "forward"
            self.legs_pub.publish(msg)
        elif(action_name=="move_backward"):
            self.say("ok")
            msg = String()
            msg.data = "backward"
            self.legs_pub.publish(msg)
        elif(action_name=="move_left"):
            self.say("ok")
            msg = String()
            msg.data = "left"
            self.legs_pub.publish(msg)
        elif(action_name=="move_right"):
            self.say("ok")
            msg = String()
            msg.data = "right"
            self.legs_pub.publish(msg)
        elif(action_name=="stop"):
            self.say("ok")
            msg = String()
            msg.data = "stop"
            self.legs_pub.publish(msg)
        elif(action_name=="dance"):
            self.say("ok")
            msg = String()
            msg.data="dance"
            self.dance_pub.publish(msg)
        elif(action_name=="namaste"):
            self.say("namaste")
        elif(action_name=="find_person_yashraj"):
            pass


def main(args=None):
    rclpy.init(args=args)
    brain_node = Brain(name="Vedanshi")
    rclpy.spin(brain_node)
    brain_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

