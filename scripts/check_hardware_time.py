"""Verify laptop receives fresh real-hardware timestamps (no motion commands)."""
import argparse
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', action='store_true', help='also require RGB, depth and calibration streams')
    parser.add_argument('--timeout', type=float, default=45.0)
    args = parser.parse_args()
    streams = [('/scan', LaserScan, 0.5), ('/odom', Odometry, 0.5)]
    if args.camera:
        streams += [('/oakd/rgb/image_raw/compressed', CompressedImage, 2.0),
                    ('/oakd/stereo/image_raw/compressedDepth', CompressedImage, 2.0),
                    ('/oakd/stereo/camera_info', CameraInfo, 2.0)]
    rclpy.init()
    node = Node('asn_clock_preflight_' + str(os.getpid()))
    latest = {}

    def receive(topic, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        latest[topic] = (stamp, time.monotonic())

    subscriptions = [node.create_subscription(kind, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data)
                     for topic, kind, _ in streams]
    healthy_since = None
    passed = False
    end = time.monotonic() + args.timeout
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            now, wall = time.monotonic(), time.time()
            healthy = all(topic in latest and abs(wall - latest[topic][0]) < limit
                          and now - latest[topic][1] < limit
                          for topic, _, limit in streams)
            if healthy:
                if healthy_since is None:
                    healthy_since = now
                if now - healthy_since >= 3:
                    passed = True
                    break
            else:
                healthy_since = None
        now, wall = time.monotonic(), time.time()
        for topic, _, limit in streams:
            if topic not in latest:
                print(f'{topic}: NO MESSAGES (limit {limit:.1f}s)', flush=True)
            else:
                stamp, received = latest[topic]
                print(f'{topic}: timestamp age {wall - stamp:+.3f}s, last received {now - received:.3f}s ago (limit {limit:.1f}s)', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not passed:
        raise SystemExit('Sensor timestamps/delivery did not stay within limits for 3 seconds. Check clocks, connectivity, topic names and sensor availability; do not navigate until scan/odom pass.')
    print('All requested streams stayed fresh for 3 seconds')


if __name__ == '__main__':
    main()
