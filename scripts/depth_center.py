#!/usr/bin/env python3
"""Print the median valid depth in the central 80 x 80 image pixels."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthCenter(Node):
    def __init__(self):
        super().__init__('asn_depth_center')
        self.last_print = 0.0
        self.last_received = time.monotonic()
        self.reported_missing = False
        self.create_timer(2.0, self.check_stream)
        self.subscription = self.create_subscription(
            Image, '/hardware_depth/image_raw', self.receive, qos_profile_sensor_data)
        self.get_logger().info('Waiting for depth images; Ctrl+C to stop')

    def check_stream(self):
        if time.monotonic() - self.last_received > 5.0 and not self.reported_missing:
            print('NO DEPTH FRAMES for 5 seconds; check camera and robot connection', flush=True)
            self.reported_missing = True

    def receive(self, msg):
        now = time.monotonic()
        self.last_received = now
        self.reported_missing = False
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        age = time.time() - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9
        if msg.encoding != '16UC1' or msg.step < msg.width * 2:
            self.get_logger().warning(f'Unsupported depth format: {msg.encoding}')
            return
        depth = np.frombuffer(msg.data, dtype='>u2' if msg.is_bigendian else '<u2')
        depth = depth.reshape(msg.height, msg.step // 2)[:, :msg.width]
        x, y = msg.width // 2, msg.height // 2
        roi = depth[max(0, y-40):min(msg.height, y+40), max(0, x-40):min(msg.width, x+40)]
        valid = roi[(roi > 0) & (roi < 10000)]
        if abs(age) > 1.0:
            print(f'STALE frame: timestamp age {age:+.2f} s', flush=True)
        elif valid.size < roi.size * 0.1:
            print('No reliable depth in the center region', flush=True)
        else:
            low, middle, high = np.percentile(valid, [10, 50, 90]) / 1000.0
            spread = high - low
            label = 'MIXED DEPTH — estimate may be unreliable' if spread > 0.30 else 'Center depth'
            print(f'{label}: {middle:.2f} m from camera '
                  f'(10–90% spread {spread:.2f} m, valid {valid.size / roi.size:.0%}, '
                  f'age {age:+.2f} s)', flush=True)


def main():
    rclpy.init()
    node = DepthCenter()
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
