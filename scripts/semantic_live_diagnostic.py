#!/usr/bin/env python3
"""Passive semantic-navigation recorder. It never publishes or sends actions."""

import argparse
import json
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


TOPICS = {
    '/semantic/perception/target_observation_json': String,
    '/semantic/perception/target_valid': Bool,
    '/semantic/perception/target_pose': PoseStamped,
    '/semantic/perception/status': String,
    '/semantic/approach/status': String,
    '/semantic/approach/goal_valid': Bool,
    '/semantic/approach/goal': PoseStamped,
    '/semantic/mission/locked_target_valid': Bool,
    '/semantic/mission/locked_target_pose': PoseStamped,
    '/semantic/mission/state': String,
    '/semantic/mission/status_json': String,
    '/semantic/nbv/execution_status': String,
    '/navigate_to_pose/_action/status': GoalStatusArray,
    '/navigate_to_pose/_action/feedback': NavigateToPose.Impl.FeedbackMessage,
    '/cmd_vel_nav': TwistStamped,
    '/cmd_vel_smoothed': TwistStamped,
    '/nav2_validation_cmd_vel': TwistStamped,
}


def stamp(message):
    header = getattr(message, 'header', None)
    if header is None:
        return None
    return header.stamp.sec + header.stamp.nanosec / 1e9


def payload(message):
    if isinstance(message, String):
        return message.data
    if isinstance(message, Bool):
        return message.data
    if isinstance(message, PoseStamped):
        p = message.pose.position
        q = message.pose.orientation
        return {'frame': message.header.frame_id, 'x': p.x, 'y': p.y, 'z': p.z,
                'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
    if isinstance(message, TwistStamped):
        linear, angular = message.twist.linear, message.twist.angular
        return {'linear': [linear.x, linear.y, linear.z],
                'angular': [angular.x, angular.y, angular.z]}
    if isinstance(message, GoalStatusArray):
        return [{'uuid': bytes(status.goal_info.goal_id.uuid).hex(),
                 'status': status.status,
                 'goal_stamp': status.goal_info.stamp.sec + status.goal_info.stamp.nanosec / 1e9}
                for status in message.status_list]
    if isinstance(message, NavigateToPose.Impl.FeedbackMessage):
        return {'uuid': bytes(message.goal_id.uuid).hex(),
                'distance_remaining': message.feedback.distance_remaining,
                'navigation_time_s': message.feedback.navigation_time.sec +
                                     message.feedback.navigation_time.nanosec / 1e9}
    return str(message)


class Recorder(Node):
    def __init__(self, output):
        super().__init__('asn_semantic_live_diagnostic')
        self.output = output
        self.started = time.time()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.recording_subscriptions = [self.create_subscription(
            message_type, topic,
            lambda message, name=topic: self.record(name, message), 10)
            for topic, message_type in TOPICS.items()]

    def record(self, topic, message):
        entry = {'received_at': time.time(), 'received_after_s': time.time() - self.started,
                 'topic': topic, 'header_stamp': stamp(message), 'value': payload(message)}
        self.output.write(json.dumps(entry, sort_keys=True) + '\n')
        self.output.flush()

    def record_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = transform.transform.translation
            self.output.write(json.dumps({'received_at': time.time(), 'topic': '/diagnostic/map_to_base_link',
                                          'header_stamp': transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9,
                                          'value': {'frame': transform.header.frame_id,
                                                    'x': t.x, 'y': t.y, 'z': t.z}}) + '\n')
            self.output.flush()
        except Exception as error:
            self.output.write(json.dumps({'received_at': time.time(), 'topic': '/diagnostic/map_to_base_link',
                                          'error': str(error)}) + '\n')
            self.output.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=90.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    with args.output.open('w', encoding='utf-8') as output:
        node = Recorder(output)
        deadline = time.monotonic() + args.seconds
        next_tf = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() >= next_tf:
                node.record_tf()
                next_tf += 2.0
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
