import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import sounddevice as sd
import numpy as np
from piper import PiperVoice
import re
import time
from utils.config_loader import get_config


class Speak(Node):
    def __init__(self):
        super().__init__("node_speak")
        config = get_config()
        self.model_path = config["piper_model_path"]

        self.voice = PiperVoice.load(self.model_path)
        self.current_model = None

        self.create_subscription(String, "piper_tts_text", self.speak_callback, 10)

        # Publisher for finished speaking
        self.finished_pub = self.create_publisher(String, "piper_tts_done", 10)
        msg = String()
        msg.data = "finished"
        self.finished_pub.publish(msg)

    def speak_callback(self, msg):
        text = msg.data
        #sentences = re.split(r'[।.,!?;\n]', text)
        sentences = re.split(r'(?<=[।.,!?;\n])', text)
        
        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            self.get_logger().info(f"Speaking: {sentence}")

            for chunk in self.voice.synthesize(sentence):
                audio = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
                sd.play(audio, samplerate=chunk.sample_rate)
                sd.wait()

        # Notify that speaking has finished
        ack = String()
        ack.data = "finished"
        time.sleep(4)
        self.finished_pub.publish(ack)
        self.get_logger().info("✅ Finished speaking")

    def model_change_callback(self, msg):
        model_path = msg.data.strip()
        try:
            self.voice = PiperVoice.load(model_path)
            self.current_model = model_path
            self.get_logger().info(f"Loaded Piper model: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model {model_path}: {e}")


def main(args=None):
    rclpy.init(args=args)
    speak_node = Speak()
    rclpy.spin(speak_node)
    speak_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

