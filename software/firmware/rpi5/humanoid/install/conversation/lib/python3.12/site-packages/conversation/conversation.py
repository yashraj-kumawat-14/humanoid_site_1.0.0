import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rapidfuzz import process, fuzz
import unicodedata
from openai import OpenAI
from utils.local_conversation_data_loader import get_local_conversation_data

def normalize_text(text: str) -> str:
    # Normalize Unicode for Devanagari
    return unicodedata.normalize("NFC", text).strip()


# === Local dataset (Q&A) in Hindi ===
local_data = get_local_conversation_data()


class ConversationNode(Node):
    def __init__(self):
        super().__init__("conversation_node")

        self.sub = self.create_subscription(
            String, "conversation_prompt", self.prompt_callback, 10
        )
        self.pub_response = self.create_publisher(String, "conversation_response", 10)
        self.pub_action = self.create_publisher(String, "humanoid_action", 10)

        self.get_logger().info("✅ Hindi Conversation Node started...")

    def prompt_callback(self, msg):
        user_prompt = normalize_text(msg.data)
        self.get_logger().info(f"📝 Prompt: {user_prompt}")

        # Step 1: Local dataset
        best_match, score, _ = process.extractOne(
            user_prompt, local_data.keys(), scorer=fuzz.token_sort_ratio
        )
        if score > 75:
            self._publish_response(local_data[best_match]["reply"])
            self.get_logger().info(f"Matched local: {best_match} ({score})")
            action_msg = String()
            action_msg.data = local_data[best_match]["action"]
            self.pub_action.publish(action_msg)
            self.get_logger().info(f"Matched action: {action_msg.data} ({score})")
            return

        # Step 2: Robot actions
        #for action, keywords in robot_actions.items():
          #  match, score, _ = process.extractOne(
           #     user_prompt, keywords, scorer=fuzz.token_sort_ratio
           # )
           # if score > 85:
           #     self.get_logger().info(f"🔥 Matched action: {action} ({score})")
            #    action_msg = String()
            #    action_msg.data = action
            #    self.pub_action.publish(action_msg)
             #   self._publish_response(f"ठीक है, अब मैं {action} कर रही हूँ।")
             #   return

        # Step 3: Fallback to GPT
        try:
            client = OpenAI()
            response = client.responses.create(
                model="gpt-4o-mini",
                input=f"यूज़र ने पूछा: {user_prompt}. इसका छोटा सा हिंदी जवाब दो। you are a female, and don't use emoji's. your response will be spoken directy by piper TTS and also make sure you don't use english words."
            )
            self._publish_response(response.output_text)
        except Exception as e:
            self.get_logger().error(f"⚠️ GPT error: {e}")
            self._publish_response("माफ़ कीजिए, अभी मैं जवाब नहीं दे सकती।")

    def _publish_response(self, text):
        msg = String()
        msg.data = text
        self.pub_response.publish(msg)
        self.get_logger().info(f"💬 Response: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = ConversationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

