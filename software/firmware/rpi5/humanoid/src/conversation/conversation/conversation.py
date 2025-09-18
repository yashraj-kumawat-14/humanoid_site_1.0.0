import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fuzzywuzzy import process, fuzz
from openai import OpenAI


# === Local dataset (Q&A) ===
local_data = {
    "hello": "Hello! I am here to help you.",
    "good morning": "Good morning! Have a great day.",
    "good evening": "Good evening! How was your day?",
    # About AI/Robot
    "what is ai": "AI stands for Artificial Intelligence — intelligence demonstrated by machines.",
    "who are you": "I am your robot assistant running on ROS2.",
    "what is your name": "My name is Vedanshi, I am your assistant.",

    # User wellbeing
    "how are you": "I'm doing great, thank you! How are you?",
    "i am fine": "Glad to hear that!",

    # Small talk
    "thank you": "You're welcome!",
    "thanks": "Anytime!",

    # Farewell
    "bye": "Goodbye! Talk to you later.",
    "good night": "Good night! Sweet dreams.",
}


# === Robot actions (English only) ===
robot_actions = {
    "move_forward": ["move forward", "go ahead", "walk forward"],
    "move_backward": ["move backward", "go back", "reverse"],
    "move_left": ["move left", "go left"],
    "move_right": ["move right", "go right"],
    "stop": ["stop", "halt", "freeze"],
    "dance": ["dance", "start dancing"],
    "greet": ["say hello", "greet", "do namaste"],
    "find_person_yashraj": ["find yashraj", "look for yashraj"]
}


class ConversationNode(Node):
    def __init__(self):
        super().__init__("conversation_node")

        # Subscribers & Publishers
        self.sub = self.create_subscription(
            String, "conversation_prompt", self.prompt_callback, 10
        )
        self.pub_response = self.create_publisher(String, "conversation_response", 10)
        self.pub_action = self.create_publisher(String, "humanoid_action", 10)

        self.get_logger().info("✅ Conversation Node started. Listening to prompts...")

    def prompt_callback(self, msg):
        user_prompt = msg.data.strip().lower()
        self.get_logger().info(f"📝 Received prompt: {user_prompt}")

        # === Step 1: Check local dataset first ===
        best_match, score = process.extractOne(
            user_prompt, local_data.keys(), scorer=fuzz.token_set_ratio
        )
        if score > 80:
            response_text = local_data[best_match]
            self.get_logger().info(f"📚 Local match: {best_match} (score {score})")
            self._publish_response(response_text)
            return

        # === Step 2: Check robot actions ===
        for action, keywords in robot_actions.items():
            best_match, score = process.extractOne(
                user_prompt, keywords, scorer=fuzz.token_set_ratio
            )
            if score > 90:
                self.get_logger().info(f"🔥 Matched action: {action} (score {score})")

                # Publish the action
                action_msg = String()
                action_msg.data = action
                self.pub_action.publish(action_msg)

                # Respond verbally
                response_text = f"Okay, performing action: {action}"
                self._publish_response(response_text)
                return

        # === Step 3: Fallback to ChatGPT ===
        self.get_logger().info("🤖 Low confidence. Asking ChatGPT...")
        try:
            client = OpenAI()
            response = client.responses.create(
                model="gpt-4o-mini",
                input=user_prompt
            )
            response_text = response.output_text
        except Exception as e:
            self.get_logger().error(f"⚠️ OpenAI API error: {e}")
            response_text = "Sorry, I could not generate a response right now."

        # Publish final response
        self._publish_response(response_text)

    def _publish_response(self, response_text):
        out_msg = String()
        out_msg.data = response_text
        self.pub_response.publish(out_msg)
        self.get_logger().info(f"💬 Published response: {response_text}")


def main(args=None):
    rclpy.init(args=args)
    node = ConversationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

