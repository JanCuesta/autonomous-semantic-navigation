#!/usr/bin/env python3
"""Simulation-only continuous wheel odometry for Step 8 / V1.

Why this exists:
Isaac Sim 6.1's in-process isaacsim_ros2_control hardware interface exposes
continuous wheel-joint positions that can jump by approximately +/-4*pi.
diff_drive_controller with position_feedback=true interprets those jumps as
real wheel motion, corrupting odometry on sufficiently long trajectories.

This node:
- subscribes only to /joint_states,
- unwraps each wheel increment sample-to-sample,
- integrates differential-drive odometry,
- publishes /odom,
- optionally publishes odom -> base_link,
- never consumes Isaac global/root pose.

It is intentionally simulation-specific. V2 physical hardware should use real
encoder feedback/base-driver odometry after hardware validation.

Example:
  python3 asn_wheel_odom_unwrapped.py --ros-args \
      -p use_sim_time:=true \
      -p wheel_radius:=0.03575 \
      -p wheel_separation:=0.23993 \
      -p publish_tf:=true
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def wrap_increment(delta: float) -> float:
    """Return the equivalent angular increment in [-pi, pi]."""
    return math.atan2(math.sin(delta), math.cos(delta))


def quaternion_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class UnwrappedWheelOdometry(Node):
    def __init__(self):
        super().__init__('asn_wheel_odom_unwrapped')

        self.declare_parameter('wheel_radius', 0.03575)
        self.declare_parameter('wheel_separation', 0.23993)
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)

        self.radius = float(self.get_parameter('wheel_radius').value)
        self.separation = float(self.get_parameter('wheel_separation').value)
        self.left_joint = str(self.get_parameter('left_joint').value)
        self.right_joint = str(self.get_parameter('right_joint').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        if self.radius <= 0.0 or self.separation <= 0.0:
            raise ValueError('wheel_radius and wheel_separation must be positive.')

        self.publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, qos_profile_sensor_data
        )

        self.prev_left = None
        self.prev_right = None
        self.prev_stamp_s = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.wrap_events_left = 0
        self.wrap_events_right = 0

        self.get_logger().info(
            'Continuous wheel odometry ready: '
            f'r={self.radius:.8f} m, b={self.separation:.8f} m, '
            f'output={self.odom_topic}, publish_tf={self.publish_tf}'
        )

    @staticmethod
    def stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def on_joint_state(self, msg: JointState):
        try:
            li = msg.name.index(self.left_joint)
            ri = msg.name.index(self.right_joint)
        except ValueError:
            return

        if li >= len(msg.position) or ri >= len(msg.position):
            return

        left = float(msg.position[li])
        right = float(msg.position[ri])
        stamp_s = self.stamp_seconds(msg.header.stamp)

        if not all(math.isfinite(v) for v in (left, right, stamp_s)):
            self.get_logger().error('Non-finite wheel position/timestamp received.')
            return

        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            self.prev_stamp_s = stamp_s
            return

        if stamp_s <= self.prev_stamp_s:
            if stamp_s < self.prev_stamp_s:
                self.get_logger().warning('Joint-state time moved backwards; resetting increment history.')
            self.prev_left = left
            self.prev_right = right
            self.prev_stamp_s = stamp_s
            return

        raw_dl = left - self.prev_left
        raw_dr = right - self.prev_right
        dl = wrap_increment(raw_dl)
        dr = wrap_increment(raw_dr)

        if abs(raw_dl - dl) > math.pi:
            self.wrap_events_left += 1
            self.get_logger().warning(
                f'Left wheel wrap corrected: raw_delta={raw_dl:+.6f} rad, '
                f'used_delta={dl:+.6f} rad, count={self.wrap_events_left}'
            )

        if abs(raw_dr - dr) > math.pi:
            self.wrap_events_right += 1
            self.get_logger().warning(
                f'Right wheel wrap corrected: raw_delta={raw_dr:+.6f} rad, '
                f'used_delta={dr:+.6f} rad, count={self.wrap_events_right}'
            )

        ds_l = self.radius * dl
        ds_r = self.radius * dr
        ds = 0.5 * (ds_l + ds_r)
        dtheta = (ds_r - ds_l) / self.separation
        dt = stamp_s - self.prev_stamp_s

        mid = self.yaw + 0.5 * dtheta
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw += dtheta

        linear = ds / dt
        angular = dtheta / dt

        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Conservative provisional planar covariance for V1 simulation.
        # These values are not a calibration claim; later estimation work can tune them.
        odom.pose.covariance[0] = 1e-4
        odom.pose.covariance[7] = 1e-4
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 4e-4

        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        odom.twist.covariance[0] = 1e-4
        odom.twist.covariance[7] = 1e-4
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 4e-4

        self.publisher.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        self.prev_left = left
        self.prev_right = right
        self.prev_stamp_s = stamp_s


def main():
    rclpy.init()
    node = UnwrappedWheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
