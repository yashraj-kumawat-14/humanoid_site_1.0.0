import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import sounddevice as sd
import numpy as np
from piper import PiperVoice
import re

class Speak(Node):
	def __init__(self):
		super().__init__("node_speak")
		
		self.voice = None # By the way choose default one from settings.json
		self.current_model = None
		
		self.create_subscription(String, "piper_tts_model_select", self.model_change_callback, 10)
		self.create_subscription(String, "piper_tts_text", self.speak_callback, 10)
		
	def speak_callback(self, msg):
		text = msg.data
		sentences = re.split(r'[।.,!?;\n]', text)
		for sentence in sentences:
			sentence = sentence.strip()
			if not sentence:
				continue
			self.get_logger().info(f"Speaking: {sentence}")
			for chunk in voice.synthesize(sentence):
				audio = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
				sd.play(audio, samplerate=chunk.sample_rate)
				sd.wait()
		
	def model_change_callback(self, msg):
		


def main(args=None):
	rclpy.init()
	speak_node = Speak()
	rclpy.spin(speak_node)
	speak_node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()		
