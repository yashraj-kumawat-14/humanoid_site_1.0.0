import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import face_recognition
import numpy as np
import os

class FaceRecognizer(Node):
    def __init__(self):
        super().__init__("node_face_recognizer")

        # ROS2 subscriptions and publisher
        self.subscription_ref = self.create_subscription(
            String,
            "face_reference",
            self.reference_callback,
            10
        )
        self.subscription_cam = self.create_subscription(
            Image,
            "camera_topic",
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(String, "face_detection", 10)

        self.bridge = CvBridge()
        self.process_this_frame = True

        # Known face variables
        self.known_face_encodings = []
        self.known_face_names = []

        self.get_logger().info("Face Recognizer Node Initialized")

    def reference_callback(self, msg):
        path = msg.data
        if not os.path.exists(path):
            self.get_logger().error(f"Reference image not found: {path}")
            return

        # Load reference face
        known_image = face_recognition.load_image_file(path)
        encodings = face_recognition.face_encodings(known_image)
        if not encodings:
            self.get_logger().error(f"No face found in reference image: {path}")
            return

        self.known_face_encodings = [encodings[0]]
        self.known_face_names = [os.path.basename(path)]
        self.get_logger().info(f"Loaded new reference face: {path}")

    def image_callback(self, msg):
        if not self.known_face_encodings:
            return  # No reference face loaded yet

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Resize for faster processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        rgb_small_frame = np.ascontiguousarray(rgb_small_frame, dtype=np.uint8)

        if self.process_this_frame:
            face_locations = face_recognition.face_locations(
                rgb_small_frame, number_of_times_to_upsample=2, model="hog"
            )
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

            found = False
            for face_encoding in face_encodings:
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding, tolerance=0.6)
                if True in matches:
                    found = True
                    break

            # Publish detection status
            status_msg = String()
            status_msg.data = "found" if found else "not found"
            self.pub.publish(status_msg)
            self.get_logger().info(f"Face detection status: {status_msg.data}")

        self.process_this_frame = not self.process_this_frame

        # Optional: draw rectangles
        for (top, right, bottom, left) in face_locations:
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

        cv2.imshow("Face Recognition", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

