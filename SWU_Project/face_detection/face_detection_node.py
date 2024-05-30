import rclpy
from rclpy.node import Node
import cv2
import yaml
import numpy as np
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import common, detect
from geometry_msgs.msg import Point
import threading

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')

        config_path = self.declare_parameter('config_path', '/home/ubuntu/SWU_Project/SWU_Project/face_detection/config/face_tracker.yaml').value
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        self.model_path = f'/home/ubuntu/SWU_Project/SWU_Project/face_detection/models/{config["detector_model"]}'
        self.detection_threshold = config['detection_threshold']

        try:
            self.interpreter = make_interpreter(self.model_path)
            self.interpreter.allocate_tensors()
            self.get_logger().info("Model successfully loaded.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            rclpy.shutdown()
            return

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera.")
            rclpy.shutdown()
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

        self.publisher_ = self.create_publisher(Point, 'face_coordinates', 10)

        self.lock = threading.Lock()
        self.frame = None
        self.running = True

        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.start()

        self.timer = self.create_timer(0.1, self.detect_faces)

    def capture_frames(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def preprocess_and_infer(self, frame):
        _, scale = common.set_resized_input(self.interpreter, (320, 320), lambda size: cv2.resize(frame, size))
        self.interpreter.invoke()
        return detect.get_objects(self.interpreter, score_threshold=self.detection_threshold, image_scale=scale)

    def detect_faces(self):
        with self.lock:
            if self.frame is None:
                return
            frame = self.frame.copy()

        faces = self.preprocess_and_infer(frame)

        if not faces:
            self.get_logger().info("No faces detected.")
        else:
            self.get_logger().info(f"Detected {len(faces)} faces.")
            self.publish_face_coordinates(frame, faces)

        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def publish_face_coordinates(self, frame, faces):
        for face in faces:
            bbox = face.bbox
            xmin = max(0, bbox.xmin)
            ymin = max(0, bbox.ymin)
            xmax = min(frame.shape[1], bbox.xmax)
            ymax = min(frame.shape[0], bbox.ymax)
            cx = (xmin + xmax) / 2
            cy = (ymin + ymax) / 2
            area = (xmax - xmin) * (ymax - ymin)  # Calculate the area of the bounding box
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 255, 0), -1)
            
            point = Point()
            point.x = float(cx)
            point.y = float(cy)
            point.z = float(area)  # Using the area as a proxy for distance
            self.publisher_.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
