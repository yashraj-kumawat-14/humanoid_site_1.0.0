import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pyaudio
import vosk
import json
import time
import numpy as np
import os
from utils.config_loader import get_config

class Listen(Node):
    def __init__(self):
        super().__init__("node_listen")
        config = get_config()

        # ROS publisher
        self.publisher_ = self.create_publisher(String, "listen_with_vosk", 10)

        # Default model path
        default_model_path = config["vosk_model_path"]#"/home/yantrigo/Dropbox/humanoid_site_1.0.0/software/firmware/rpi5/humanoid/src/ears/ears/vosk_models/vosk-model-small-en-us-0.15"
        self.model_path = os.path.expanduser(default_model_path)
        self.model = vosk.Model(self.model_path)

        # PyAudio setup
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=4000
        )
        self.stream.start_stream()

        self.rec = vosk.KaldiRecognizer(self.model, 16000)

        # Silence detection params
        self.SILENCE_LIMIT = 3.0  # seconds
        self.last_speech_time = time.time()
        self.partial_text = ""

        # ROS timer to poll microphone continuously
        self.timer = self.create_timer(0.1, self.listen_callback)

        self.get_logger().info(f"Speech-to-Text node started with model {self.model_path}")

    def is_silent(self, chunk, threshold=500):
        """Check if audio chunk is silent based on RMS"""
        data = np.frombuffer(chunk, dtype=np.int16)
        rms = np.sqrt(np.mean(np.square(data)))
        return rms < threshold

    def listen_callback(self):
        data = self.stream.read(4000, exception_on_overflow=False)

        # Update last speech time if not silent
        if not self.is_silent(data):
            self.last_speech_time = time.time()

        # Feed audio to recognizer
        if self.rec.AcceptWaveform(data):
            result = json.loads(self.rec.Result())
            self.partial_text += " " + result.get("text", "")

        # Publish full sentence when silence exceeds limit
        if (time.time() - self.last_speech_time) > self.SILENCE_LIMIT and self.partial_text.strip():
            msg = String()
            msg.data = self.partial_text.strip()
            self.publisher_.publish(msg)
            self.get_logger().info(f"🗣️ Published: {msg.data}")

            # Reset for next sentence
            self.partial_text = ""
            self.rec = vosk.KaldiRecognizer(self.model, 16000)
            self.last_speech_time = time.time()

    def model_change_callback(self, msg: String):
        new_model_path = os.path.expanduser(msg.data.strip())
        try:
            self.get_logger().info(f"Loading new Vosk model from: {new_model_path}")
            new_model = vosk.Model(new_model_path)
            self.model = new_model
            self.model_path = new_model_path
            self.rec = vosk.KaldiRecognizer(self.model, 16000)
            self.partial_text = ""
            self.get_logger().info(f"✅ Successfully switched to Vosk model: {new_model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model {new_model_path}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Listen()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop_stream()
        node.stream.close()
        node.p.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

