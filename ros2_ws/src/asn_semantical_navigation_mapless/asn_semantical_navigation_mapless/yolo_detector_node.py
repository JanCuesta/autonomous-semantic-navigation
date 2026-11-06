"""Optional RGB-only YOLO detection for the currently requested COCO class.

Requires ultralytics and cv_bridge at runtime. Detection is NOT localization;
no target approach is sent. No hallucinated 'black/color' attribute checks.
"""

import json
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

from .target_filter import TargetFilter


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.declare_parameters('', [
            ('image_topic', '/rgb'),
            ('compressed_image', False),
            ('model', 'yolo11n.pt'),
            ('device', 'cpu'),
            ('confidence_threshold', 0.55),
            ('confirmation_frames', 3),
            ('processing_hz', 2.0),
        ])
        try:
            from ultralytics import YOLO
            from cv_bridge import CvBridge
        except ImportError as exc:
            raise RuntimeError(
                'YOLO requires ultralytics and ros-jazzy-cv-bridge. '
                'See V3 implementation README.') from exc
        model_path = str(self.get_parameter('model').value)
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.available_classes = set(self.model.names.values())
        self.filter = TargetFilter(
            int(self.get_parameter('confirmation_frames').value),
            float(self.get_parameter('confidence_threshold').value))
        self.last_processed = 0.0
        self.target_valid = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        
        self.found_pub = self.create_publisher(
            Bool,
            '/semantic/nbv/target_found', 
            10
        )
        
        self.detections_pub = self.create_publisher(
            String, 
            '/semantic/search/detections_json', 
            10
            )

        self.annotated_pub = self.create_publisher(
            Image,
            '/semantic/perception/annotated_image',
            qos_profile_sensor_data
        )
        
        self.create_subscription(String, '/semantic/search/target_class',
                                 self.on_target, qos)
        
        image_type = (CompressedImage if self.get_parameter('compressed_image').value
                      else Image)
        self.create_subscription(image_type,
                                 str(self.get_parameter('image_topic').value),
                                 self.on_image, qos_profile_sensor_data)
        
        self.get_logger().info(
            f'YOLO ready on {self.get_parameter("image_topic").value}. '
            'Select target via search_command_node.')

    def on_target(self, msg):
        self.target_valid = msg.data in self.available_classes
        if not self.target_valid:
            self.get_logger().error(
                f'Target {msg.data!r} not in model class list; cannot search')
            self.filter.set_target('')
            self.found_pub.publish(Bool(data=False))
            return
        self.filter.set_target(msg.data)
        self.found_pub.publish(Bool(data=False))
        self.get_logger().info(f'Waiting for camera evidence of {msg.data!r}')

    def on_image(self, msg):
        if not self.target_valid or not self.filter.target:
            return
        interval = 1 / max(0.1, self.get_parameter('processing_hz').value)
        now = time.monotonic()
        if now - self.last_processed < interval:
            return
        self.last_processed = now
        try:
            if isinstance(msg, CompressedImage):
                image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                                     cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError('Could not decode compressed RGB image')
            else:
                image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            annotated = image.copy()
            results = self.model.predict(
                source=image, verbose=False,
                device=self.get_parameter('device').value,
                conf=self.get_parameter('confidence_threshold').value)
            boxes = results[0].boxes
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return
        confidence = {}
        matches = []
        for box in boxes:
            label = self.model.names[int(box.cls[0])]
            score = float(box.conf[0])
            confidence[label] = max(confidence.get(label, 0.0), score)
            if label == self.filter.target:
                matches.append({
                    'class': label, 'confidence': round(score, 3),
                    'bbox_xyxy': [round(float(v), 1)
                                  for v in box.xyxy[0].tolist()]})
        #Create the BoundignBox for Rviz
        for detection in matches:
            x1, y1, x2, y2 = detection['bbox_xyxy']

            #Find corresponding pixels:
            x1 = int(round(x1))
            x2 = int(round(x2))
            y1 = int(round(y1))
            y2 = int(round(y2))

            #Draw the rectangle:
            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                (0, 255, 0), # box color (green)
                2 #line thickness
            )
            # Class + confidence
            label = f"{detection['class']} {detection['confidence']:.2f}"

            # Add the text to the BB
            if y1 >= 25:
                label_y = y1 - 5
            else:
                label_y = y2 + 20

            cv2.putText(
                annotated,
                label,
                (x1, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        annotated_msg = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding='bgr8'
        )
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        just_confirmed = not self.filter.confirmed and bool(matches)
        found = self.filter.update(confidence)
        self.detections_pub.publish(String(data=json.dumps({
            'target': self.filter.target,
            'frame_id': msg.header.frame_id,
            'timestamp_sec': msg.header.stamp.sec,
            'timestamp_nanosec': msg.header.stamp.nanosec,
            'image_width': image.shape[1],
            'image_height': image.shape[0],
            'detections': matches,
            'confirmation_count': self.filter.count,
            'confirmed': found,
        })))
        if found and just_confirmed:
            self.get_logger().warning(
                f'TARGET_DETECTED: {self.filter.target}, '
                f'confirmed over {self.filter.required_frames} frames. '
                'No depth-based 3D localization has been performed.')
        self.found_pub.publish(Bool(data=found))


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
