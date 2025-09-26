import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
from utils.config_loader import get_config
import datetime

hindi_months = {
    1: "जनवरी", 2: "फ़रवरी", 3: "मार्च", 4: "अप्रैल",
    5: "मई", 6: "जून", 7: "जुलाई", 8: "अगस्त",
    9: "सितंबर", 10: "अक्टूबर", 11: "नवंबर", 12: "दिसंबर"
}

class Brain(Node):
    def __init__(self, name="राधा"):
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
        if(not user_text.startswith(self.name)):
            return
        
        user_text = re.sub(rf'^\s*{re.escape(self.name)}\s*', '', user_text, count=1)
        
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
            msg = String()
            msg.data = "forward"
            self.legs_pub.publish(msg)
        elif(action_name=="move_backward"):
            msg = String()
            msg.data = "backward"
            self.legs_pub.publish(msg)
        elif(action_name=="move_left"):
            msg = String()
            msg.data = "left"
            self.legs_pub.publish(msg)
        elif(action_name=="move_right"):
            msg = String()
            msg.data = "right"
            self.legs_pub.publish(msg)
        elif(action_name=="stop"):
            msg = String()
            msg.data = "stop"
            self.legs_pub.publish(msg)
        elif(action_name=="do_dance"):
            msg = String()
            msg.data="dance"
            self.dance_pub.publish(msg)
        elif(action_name=="say_name"):
            msg = String()
            msg.data=f"मेरा नाम {self.name} है. मैं आपकी क्या मदद कर सकती हूँ?"
        elif(action_name=="do_bye"):
            msg = String()
            msg.data="bye"
            # publish to hands
        elif(action_name=="do_namaste"):
            pass
        elif(action_name=="get_date"):
            now = datetime.datetime.now()
            day = now.day
            month = hindi_months[now.month]
            year = now.year
            todays_date = f"{day} {month} {year}"
            text = f"आज {todays_date} है"
            self.say(text)
        elif(action_name=="none"):
            pass
        elif(action_name=="get_time"):
            now = datetime.datetime.now()
            hour_24 = now.hour
            minute = now.minute
            
            if 0 <= hour_24 < 12:
                period = "सुबह"  # morning
                hour_12 = hour_24 if hour_24 != 0 else 12
            elif 12 <= hour_24 < 16:
                period = "दोपहर"  # afternoon
                hour_12 = hour_24 - 12 if hour_24 > 12 else 12
            elif 16 <= hour_24 < 20:
                period = "शाम"  # evening
                hour_12 = hour_24 - 12
            else:
                period = "रात्रि"  # night
                hour_12 = hour_24 - 12 if hour_24 > 12 else 12
                
            if minute == 0:
                time_str = f"अभि {hour_12} बजे {period} है"
            else:
                time_str = f"अभि {period} के {hour_12} बजकर {minute} मिनट हुए है"
            self.say(f"{time_str}")
            
        elif(action_name=="find_yashraj"):
            pass
        elif(action_name=="shutdown"):
            pass


def main(args=None):
    config = get_config()
    name = config["humanoid_name"]
    rclpy.init(args=args)
    brain_node = Brain(name=name)
    rclpy.spin(brain_node)
    brain_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

