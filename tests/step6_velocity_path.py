#!/usr/bin/env python3
"""Exercise the active Isaac differential drive using sim-time stamped commands.

Run in a system Jazzy terminal with Isaac playing and both controllers active.
Moves about 12 cm forward, 6 cm backward, then turns at 0.2 rad/s.
Writes observations; zero commands are sent on exit. Does not launch publishers
for joint states or robot transforms. Isaac pose observation is a separate check.
"""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='results/step6/velocity_path.json')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('asn_step6_velocity_test')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    pub = node.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
    state = {'joint': None, 'odom': None, 'tf': [], 'scan_count': 0}
    records = []
    phase = 'discovery'

    def now():
        return node.get_clock().now().nanoseconds / 1e9

    def joints(msg):
        state['joint'] = {'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                          'position': dict(zip(msg.name, msg.position)),
                          'velocity': dict(zip(msg.name, msg.velocity))}

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        state['odom'] = {'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                         'frame': msg.header.frame_id, 'child': msg.child_frame_id,
                         'position': [p.x, p.y, p.z], 'orientation': [q.x, q.y, q.z, q.w],
                         'linear': msg.twist.twist.linear.x, 'angular': msg.twist.twist.angular.z}

    def tf(msg):
        state['tf'] = [{'parent': t.header.frame_id, 'child': t.child_frame_id,
                        'stamp': t.header.stamp.sec + t.header.stamp.nanosec / 1e9,
                        'xy': [t.transform.translation.x, t.transform.translation.y]}
                       for t in msg.transforms]

    def scan(_):
        state['scan_count'] += 1

    node.create_subscription(JointState, '/joint_states', joints, 10)
    node.create_subscription(Odometry, '/diff_drive_controller/odom', odom, 10)
    node.create_subscription(TFMessage, '/tf', tf, 10)
    from rclpy.qos import qos_profile_sensor_data
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)

    def command(v, w):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        pub.publish(msg)

    schedule = [('baseline', 1.5, 0.0, 0.0), ('forward', 3.0, .04, 0.0),
                ('forward_timeout', 2.0, None, None), ('reverse', 2.0, -.03, 0.0),
                ('reverse_timeout', 2.0, None, None), ('turn', 2.0, 0.0, .2),
                ('turn_timeout', 2.0, None, None), ('stop', 1.0, 0.0, 0.0)]
    try:
        deadline = time.monotonic() + 15
        while not (now() > 0 and state['joint'] and state['odom'] and pub.get_subscription_count()):
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() > deadline:
                raise RuntimeError('Clock, joint feedback, odometry, or command subscriber missing')
        for phase, duration, v, w in schedule:
            start = now()
            deadline = time.monotonic() + 45
            last_pub = last_sample = -1.0
            print(f'{phase}: sim_start={start:.3f}, command=({v}, {w}), duration={duration}', flush=True)
            while now() - start < duration:
                rclpy.spin_once(node, timeout_sec=.01)
                wall = time.monotonic()
                if wall > deadline:
                    raise RuntimeError('Simulation clock stalled')
                if v is not None and wall - last_pub >= .05:
                    command(v, w)
                    last_pub = wall
                if now() - last_sample >= .05:
                    records.append({'phase': phase, 'time': now(), 'elapsed': now() - start,
                                    **json.loads(json.dumps(state))})
                    last_sample = now()
    finally:
        for _ in range(10):
            command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=.02)
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(json.dumps(records, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
        print(f'Saved {len(records)} samples to {args.output}', flush=True)


if __name__ == '__main__':
    main()
