import rclpy
from rclpy.node import Node
import cv2
import yaml
import numpy as np
import threading
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import common, detect
from geometry_msgs.msg import Point
from picamera.array import PiRGBArray
from picamera import PiCamera

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')

        # Load configuration
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

        # Initialize camera
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.raw_capture = PiRGBArray(self.camera, size=(640, 480))
        
        self.publisher_ = self.create_publisher(Point, 'face_coordinates', 10)

        self.lock = threading.Lock()
        self.frame = None
        self.running = True

        # Start camera capture thread
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.start()

        # Create a timer for face detection
        self.timer = self.create_timer(0.1, self.detect_faces)

    def capture_frames(self):
        for frame in self.camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True):
            with self.lock:
                self.frame = frame.array
            self.raw_capture.truncate(0)
            if not self.running:
                break

    def preprocess_and_infer(self, frame):
        _, scale = common.set_resized_input(self.interpreter, (320, 320), lambda size: cv2.resize(frame, size))
        self.interpreter.invoke()
        return detect.get_objects(self.interpreter, score_threshold=self.detection_threshold, image_scale=scale)

    def draw_bounding_boxes(self, frame, faces):
        for face in faces:
            bbox = face.bbox
            xmin = max(0, bbox.xmin)
            ymin = max(0, bbox.ymin)
            xmax = min(frame.shape[1], bbox.xmax)
            ymax = min(frame.shape[0], bbox.ymax)
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(frame, f'Score: {face.score:.2f}', (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            self.get_logger().info(f"Face detected - Score: {face.score:.2f}, Bbox: {xmin}, {ymin}, {xmax}, {ymax}")

            point = Point()
            point.x = (xmin + xmax) / 2
            point.y = (ymin + ymax) / 2
            point.z = bbox.area()
            self.publisher_.publish(point)

    def detect_faces(self):
        with self.lock:
            if self.frame is None:
                self.get_logger().error("Failed to capture frame from camera.")
                return
            frame = self.frame.copy()
        
        faces = self.preprocess_and_infer(frame)

        if not faces:
            self.get_logger().info("No faces detected.")
        else:
            self.get_logger().info(f"Detected {len(faces)} faces.")
            self.draw_bounding_boxes(frame, faces)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False
            self.camera.close()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
