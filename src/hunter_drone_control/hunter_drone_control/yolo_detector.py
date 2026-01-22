import time
from typing import List

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

# CV & ML
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO


class YoloDetector(Node):
    '''
    Docstring for YoloDetector
    '''
    def __init__(self) -> None:
        super().__init__('yolo_detector')

        # Params
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_classes', [0, 2, 5, 7]) # Coco Datasets - person, car ,bus , truck
        self.declare_parameter('enable_debug_image', True)

        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.target_classes = list(self.get_parameter('target_classes').value)
        self.enable_debug = self.get_parameter('enable_debug_image').value

        # Initialization
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"Model Loaded: {self.model_path}")
        except Exception as e:
            self.get_logger().fatal(f'Failed to load model {e}')
            raise RuntimeError('Fail to load model {e}')
        
        self.bridge = CvBridge()

        # Performance monitoring
        self.last_time = time.time()
        self.frame_count = 0

        # QoS Configuration
        # Keep depth low (1) to always process the latest frame and minimize latency.
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,

        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/vision/detections',
            qos_reliable
        )

        # Debug visualization
        self.debug_pub = self.create_publisher(
            Image,
            '/vision/debug_image',
            qos_sensor_data
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_sensor_data
        )

        self.get_logger().info('Detector node initialized')

    def image_callback(self, msg: Image) -> None:
        """
        Process incoming camera frames and publish detections
        
        Args: Ros image message from camera
        """
        start_time = time.time()

        # Convert ROS image to cv
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Conversion failed: {e}')
            return
        
        # Inference
        try:
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            # Publish empty detection for handle no data
            self.detection_pub.publish(Detection2DArray(header=msg.header))

        # Parse results
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        # Copy for visualization
        debug_image = cv_image.copy() if self.enable_debug else None

        # Process
        if results and len(results) > 0:
            result = results[0]

            if result.boxes is not None:
                for box in result.boxes:
                    # Extract and convert to int classID
                    cls_id = int(box.cls[0].cpu().numpy())

                    # Filter by target classes
                    if cls_id not in self.target_classes:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    conf = float(box.conf[0].cpu().numpy())

                    # Create detection2d message
                    detection = Detection2D()
                    detection.header = msg.header

                    # Define centers
                    detection.bbox.center.position.x = float((x1 + x2) / 2.0)
                    detection.bbox.center.position.y = float((y1 + y2) / 2.0)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)

                    # Classification Hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls_id)
                    hypothesis.hypothesis.score = conf
                    detection.results.append(hypothesis)
                    
                    detection_array.detections.append(detection)

                    # Visualization
                    if self.enable_debug and debug_image is not None:
                        label = f"{self.model.names[cls_id]} {conf:.2f}"
                        self._draw_bbox(debug_image, x1, y1, x2, y2, label)
        
        self.detection_pub.publish(detection_array)

        if self.enable_debug and debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except CvBridgeError as e:
                self.get_logger().warn(f'Failed to publish debug image: {e}')

        self._log_performance(start_time) # Performance monitoring

    def _draw_bbox(self, img: np.ndarray, x1: int, y1: int, x2: int, y2: int, label: str) -> None:
        """
        Helper function to draw bounding boxes and labels on the debug image.

        Args:
            img: Image array
            x1, y1, x2, y2: Bounding box coordinates
            label: Text label to display
        """
        color = (0, 255, 0) # Green - BGR
        thickness = 2
        
        # Draw rectangle
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
        
        # Draw label background
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        cv2.rectangle(
            img, 
            (int(x1), int(y1) - text_height - baseline - 5), 
            (int(x1) + text_width, int(y1)), 
            color, 
            -1
        )
        
        # Draw text
        cv2.putText(
            img, 
            label, 
            (int(x1), int(y1) - baseline - 2), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.5, 
            (0, 0, 0), # Black text
            1
        )

    def _log_performance(self, start_time: float) -> None:
        """
        Calculate and log FPS with throttling to prevent log flooding.
        """
        self.frame_count += 1
        curr_time = time.time()
        elapsed = curr_time - self.last_time
        
        # Log every second
        if elapsed > 1.0:
            fps = self.frame_count / elapsed
            inference_time = (curr_time - start_time) * 1000 # ms
            self.get_logger().info(
                f'Performance: {fps:.1f} FPS | Inference: {inference_time:.1f} ms | Detections: {len(self.detection_pub.subscriptions)}' 
            )
            self.frame_count = 0
            self.last_time = curr_time

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down YOLO Detector')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    detector = YoloDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()     