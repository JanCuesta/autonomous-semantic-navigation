"""Mapless RGB-D target approach preview: keep original ROS interfaces.

Uses live LiDAR for local clearance, target odom pose for standoff, and never
publishes wheel commands. The semantic mission manager owns action execution.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .mapless_core import (approach_candidates, relative_scan_points,
                           sector_clearance, segment_clear, wrap)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y),
                      1-2*(q.y*q.y+q.z*q.z))


class MaplessApproach(Node):
    def __init__(self):
        super().__init__('approach_goal_node')
        self.declare_parameters('', [
            ('robot_frame', 'base_link'),
            ('odom_frame', 'odom'),
            ('scan_topic', '/scan'),
            ('stand_off_m', 0.60),
            ('clearance_m', 0.34),
            ('target_timeout_s', 2.5),
            ('locked_target_max_age_s', 15.0),
            ('scan_timeout_s', 0.65),
            ('max_approach_length_m', 2.80),
        ])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.scan, self.scan_at = None, 0.0
        self.target, self.target_at, self.target_valid = None, 0.0, False
        self.locked, self.locked_at, self.locked_valid = None, 0.0, False
        self.state = 'IDLE'
        self.last_status = ''
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, self.p('scan_topic'),
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped,
                                 '/semantic/perception/target_pose',
                                 self.on_target, 10)
        self.create_subscription(Bool, '/semantic/perception/target_valid',
                                 self.on_target_valid, 10)
        self.create_subscription(PoseStamped,
                                 '/semantic/mission/locked_target_pose',
                                 self.on_locked, qos)
        self.create_subscription(Bool,
                                 '/semantic/mission/locked_target_valid',
                                 self.on_locked_valid, qos)
        self.create_subscription(String, '/semantic/mission/state',
                                 self.on_state, qos)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/semantic/approach/goal', 10)
        self.locked_pub = self.create_publisher(
            PoseStamped, '/semantic/approach/locked_goal', 10)
        self.valid_pub = self.create_publisher(
            Bool, '/semantic/approach/goal_valid', 10)
        self.status_pub = self.create_publisher(
            String, '/semantic/approach/status', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic/approach/candidates', 10)
        self.create_service(Trigger, '/semantic/approach/send_goal',
                            self.reject_direct_execution)
        self.create_timer(0.3, self.plan)

    def p(self, name):
        return self.get_parameter(name).value

    def on_scan(self, msg):
        self.scan, self.scan_at = msg, time.monotonic()

    def on_target(self, msg):
        self.target, self.target_at = msg, time.monotonic()

    def on_target_valid(self, msg):
        self.target_valid = bool(msg.data)

    def on_locked(self, msg):
        self.locked, self.locked_at = msg, time.monotonic()

    def on_locked_valid(self, msg):
        self.locked_valid = bool(msg.data)
        if not self.locked_valid:
            self.locked = None

    def on_state(self, msg):
        self.state = msg.data.strip()

    def reject_direct_execution(self, request, response):
        response.success = False
        response.message = 'Mission supervisor exclusively owns approach action'
        return response

    def report(self, message):
        self.status_pub.publish(String(data=message))
        if message != self.last_status:
            self.get_logger().info(message)
            self.last_status = message

    def fail(self, reason):
        self.valid_pub.publish(Bool(data=False))
        self.report(reason)

    def pose_and_scan(self):
        odom_robot = self.tf.lookup_transform(
            self.p('odom_frame'), self.p('robot_frame'), Time())
        robot = odom_robot.transform.translation
        robot_yaw = yaw_of(odom_robot.transform.rotation)
        sensor = self.tf.lookup_transform(
            self.p('robot_frame'), self.scan.header.frame_id, Time())
        laser = sensor.transform.translation
        laser_yaw = yaw_of(sensor.transform.rotation)
        points = list(relative_scan_points(
            self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
            self.scan.range_min, self.scan.range_max,
            laser.x, laser.y, laser_yaw))
        return robot.x, robot.y, robot_yaw, laser_yaw, points

    def publish_marker(self, goal):
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker = Marker()
        marker.header = goal.header
        marker.ns = 'mapless_approach'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.24
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.g = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(MarkerArray(markers=[clear, marker]))

    def plan(self):
        # Lock-specific preview must follow the mission manager's state.
        if self.state not in ('IDLE', 'WAITING_APPROACH'):
            return
        locked = self.state == 'WAITING_APPROACH'
        target = self.locked if locked else self.target
        valid = self.locked_valid if locked else self.target_valid
        timestamp = self.locked_at if locked else self.target_at
        max_age = self.p('locked_target_max_age_s') if locked else self.p(
            'target_timeout_s')
        if (not valid or target is None or
                time.monotonic()-timestamp > max_age):
            self.fail('WAITING: recent validated RGB-D target')
            return
        if target.header.frame_id != self.p('odom_frame'):
            self.fail('REJECTED: RGB-D target must be in odom')
            return
        if (self.scan is None or time.monotonic()-self.scan_at >
                self.p('scan_timeout_s')):
            self.fail('WAITING: fresh /scan')
            return
        try:
            rx, ry, robot_yaw, laser_yaw, points = self.pose_and_scan()
        except TransformException:
            self.fail('WAITING: odom/base/laser TF')
            return
        tx, ty = target.pose.position.x, target.pose.position.y
        if not all(map(math.isfinite, (rx, ry, tx, ty))):
            self.fail('REJECTED: non-finite target')
            return
        options = []
        for x, y, yaw in approach_candidates(rx, ry, tx, ty,
                                             self.p('stand_off_m')):
            dx, dy = x-rx, y-ry
            distance = math.hypot(dx, dy)
            if distance > self.p('max_approach_length_m'):
                continue
            lx = math.cos(robot_yaw)*dx + math.sin(robot_yaw)*dy
            ly = -math.sin(robot_yaw)*dx + math.cos(robot_yaw)*dy
            direction = math.atan2(ly, lx)
            available = sector_clearance(
                self.scan.ranges, self.scan.angle_min,
                self.scan.angle_increment, self.scan.range_min,
                self.scan.range_max, wrap(direction-laser_yaw),
                half_width=0.12)
            # Sensor must see the whole straight path; never project through
            # unseen/occluded space. Executor checks again during motion.
            if (available >= distance + self.p('clearance_m') and
                    segment_clear(points, lx, ly, self.p('clearance_m'))):
                options.append((distance, x, y, yaw))
        if not options:
            self.fail('NO_GOAL: no sensor-observed straight approach; do not '
                      'assume unseen space is free')
            return
        distance, x, y, heading = min(options)
        goal = PoseStamped()
        goal.header.frame_id = self.p('odom_frame')
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(heading/2)
        goal.pose.orientation.w = math.cos(heading/2)
        self.goal_pub.publish(goal)
        if locked:
            self.locked_pub.publish(goal)
        self.valid_pub.publish(Bool(data=True))
        self.publish_marker(goal)
        self.report(f'PREVIEW: odom=({x:.2f},{y:.2f}) '
                    f'length={distance:.2f}m, sensor-checked')


def main(args=None):
    rclpy.init(args=args)
    node = MaplessApproach()
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
