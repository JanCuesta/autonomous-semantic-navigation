"""Decode the hardware RGB stream locally for a stationary RViz preview."""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class RgbPreview(Node):
    def __init__(self):
        super().__init__('hardware_rgb_preview')
        self.declare_parameter('image_topic', '/oakd/rgb/image_raw/compressed')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/semantic/rgb_preview/image_raw', qos_profile_sensor_data)
        self.compressed_publisher = self.create_publisher(
            CompressedImage, '/semantic/rgb_input/compressed', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            CompressedImage, self.get_parameter('image_topic').value, self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        # One robot-side subscription feeds both the detector and RViz locally.
        if self.compressed_publisher.get_subscription_count():
            self.compressed_publisher.publish(msg)
        if not self.publisher.get_subscription_count():
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Unable to decode RGB frame')
            return
        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header = msg.header
        self.publisher.publish(image)


def main():
    rclpy.init()
    node = RgbPreview()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
