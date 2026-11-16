"""Persistent NavigateToPose client for supervised NAV-001 translation tests.

Runs a fixed preflight, sends at most one goal at a time, records feedback,
every velocity-pipeline stage and terminal status, and evaluates the final
stationary pose against the active Nav2 goal tolerances.

Guards are independent of each other: a simulated-time budget for Nav2, a
wall-clock guard, a stalled-/clock guard and a clock-reset guard. Any guard
cancels the goal and then waits for a confirmed terminal status.

Examples:
  python3 diagnostics/nav001_action_recorder.py OUT --dry-run
  python3 diagnostics/nav001_action_recorder.py OUT --count 3
  python3 diagnostics/nav001_action_recorder.py OUT --goal 0.0 0.6 90
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_action_status_default,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # pragma: no cover - older Nav2
    CollisionMonitorState = None


VELOCITY_TOPICS = ('/cmd_vel_nav', '/cmd_vel_smoothed',
                   '/diff_drive_controller/cmd_vel')
ACTIVE_STATUSES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                   GoalStatus.STATUS_CANCELING)
TERMINAL_STATUSES = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                     GoalStatus.STATUS_ABORTED)
STATUS_NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def uuid_hex(goal_id):
    return ''.join(f'{value:02x}' for value in goal_id.uuid)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Grid:
    """World-to-cell lookup for an OccupancyGrid, including origin yaw."""

    def __init__(self, msg):
        info = msg.info
        self.res = info.resolution
        self.width = info.width
        self.height = info.height
        self.ox = info.origin.position.x
        self.oy = info.origin.position.y
        self.oyaw = yaw_of(info.origin.orientation)
        self.data = msg.data

    def value(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        c, s = math.cos(-self.oyaw), math.sin(-self.oyaw)
        i = math.floor((c * dx - s * dy) / self.res)
        j = math.floor((s * dx + c * dy) / self.res)
        if 0 <= i < self.width and 0 <= j < self.height:
            return self.data[j * self.width + i]
        return None

    def worst_in_disc(self, x, y, radius):
        """Max value in a disc; unknown/outside cells are returned as None."""
        worst = 0
        steps = int(math.ceil(radius / self.res))
        for di in range(-steps, steps + 1):
            for dj in range(-steps, steps + 1):
                ox, oy = di * self.res, dj * self.res
                if ox * ox + oy * oy > radius * radius:
                    continue
                v = self.value(x + ox, y + oy)
                if v is None or v < 0:
                    return None
                worst = max(worst, v)
        return worst


class Recorder(Node):
    def __init__(self, args):
        super().__init__('nav001_action_recorder', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_msg = None
        self.costmap_msg = None
        self.status_msg = None
        self.odom = None
        self.odom_wall = None
        self.scan_min = None
        self.scan_wall = None
        self.last_cmd = {topic: None for topic in VELOCITY_TOPICS}
        self.recording = None
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_costmap, latched)
        self.create_subscription(GoalStatusArray,
                                 '/navigate_to_pose/_action/status',
                                 self.on_status,
                                 qos_profile_action_status_default)
        self.create_subscription(Odometry, '/odom', self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        for topic in VELOCITY_TOPICS:
            self.create_subscription(
                TwistStamped, topic,
                lambda msg, t=topic: self.on_cmd(t, msg),
                qos_profile_sensor_data)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState,
                                     '/collision_monitor_state',
                                     self.on_collision_state, 10)

    # ---------------------------------------------------------------- inputs
    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_map(self, msg):
        self.map_msg = msg

    def on_costmap(self, msg):
        self.costmap_msg = msg

    def on_status(self, msg):
        self.status_msg = msg
        if self.recording is not None:
            for status in msg.status_list:
                if uuid_hex(status.goal_info.goal_id) == self.recording['uuid']:
                    self.recording['status_topic'] = status.status

    def on_odom(self, msg):
        self.odom = msg
        self.odom_wall = time.monotonic()
        self.velocity_row('/odom', stamp_s(msg.header.stamp),
                          msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_scan(self, msg):
        valid = [r for r in msg.ranges
                 if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        self.scan_min = min(valid) if valid else None
        self.scan_wall = time.monotonic()
        rec = self.recording
        if rec is not None and rec['phase'] in ('goal', 'settle') \
                and self.scan_min is not None:
            rec['scan_min'] = min(rec['scan_min'], self.scan_min)

    def on_cmd(self, topic, msg):
        self.last_cmd[topic] = (time.monotonic(), msg.twist.linear.x,
                                msg.twist.angular.z)
        self.velocity_row(topic, stamp_s(msg.header.stamp),
                          msg.twist.linear.x, msg.twist.angular.z)

    def on_collision_state(self, msg):
        rec = self.recording
        if rec is not None and msg.action_type != 0:
            rec['collision_monitor_actions'].append(
                [self.sim_now(), int(msg.action_type), msg.polygon_name])

    def velocity_row(self, topic, header_s, vx, wz):
        rec = self.recording
        if rec is None:
            return
        wall = time.monotonic() - rec['wall_start']
        sim = self.sim_now()
        rec['vel_csv'].writerow([f'{wall:.4f}', f'{sim:.4f}',
                                 f'{header_s:.4f}', rec['phase'], topic,
                                 f'{vx:.5f}', f'{wz:.5f}'])
        stats = rec['vel_stats'].setdefault(topic, {
            'n': 0, 'max_abs_vx': 0.0, 'max_abs_wz': 0.0})
        if rec['phase'] == 'goal':
            stats['n'] += 1
            stats['max_abs_vx'] = max(stats['max_abs_vx'], abs(vx))
            stats['max_abs_wz'] = max(stats['max_abs_wz'], abs(wz))
        stats['last_vx'], stats['last_wz'] = vx, wz
        stats['last_phase'] = rec['phase']
        if topic == '/diff_drive_controller/cmd_vel':
            rec['last_base_cmd'] = (sim, vx)
        if topic == '/odom' and rec['phase'] == 'goal':
            base = rec.get('last_base_cmd')
            if base and abs(base[1]) > 0.03 and abs(vx) < 0.005 \
                    and rec.get('last_odom_sim') is not None:
                rec['stall_s'] += max(0.0, sim - rec['last_odom_sim'])
            rec['last_odom_sim'] = sim

    # ------------------------------------------------------------- helpers
    def spin_for(self, executor, wall_s, until=None):
        deadline = time.monotonic() + wall_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def pose(self, frame='base_link'):
        tf = self.tf_buffer.lookup_transform('map', frame, Time())
        t = tf.transform
        return (t.translation.x, t.translation.y, yaw_of(t.rotation),
                stamp_s(tf.header.stamp))

    def active_goals(self):
        if self.status_msg is None:
            return []
        return [(uuid_hex(s.goal_info.goal_id), STATUS_NAMES[s.status])
                for s in self.status_msg.status_list
                if s.status in ACTIVE_STATUSES]

    # ------------------------------------------------------------ preflight
    def preflight(self, executor):
        a = self.args
        checks = {}
        self.spin_for(executor, 3.0, lambda: self.map_msg is not None
                      and self.costmap_msg is not None
                      and self.odom is not None and self.scan_min is not None)
        clock0, wall0 = self.sim_now(), time.monotonic()
        self.spin_for(executor, 2.0)
        clock1 = self.sim_now()
        rtf = (clock1 - clock0) / max(1e-6, time.monotonic() - wall0)
        checks['clock_advancing'] = {'ok': clock0 > 0 and clock1 > clock0,
                                     'sim_s': clock1, 'rtf': round(rtf, 3)}
        checks['action_server'] = {
            'ok': self.client.wait_for_server(timeout_sec=5.0)}
        self.spin_for(executor, 1.0)
        active = self.active_goals()
        checks['no_active_goals'] = {'ok': not active, 'active': active}
        try:
            x, y, yaw, stamp = self.pose()
            age = clock1 - stamp
            checks['tf_map_base_link'] = {
                'ok': age < 2.0, 'pose': [x, y, yaw], 'age_sim_s': age}
        except Exception as exc:  # noqa: BLE001 - recorded as preflight evidence
            checks['tf_map_base_link'] = {'ok': False, 'error': str(exc)}
        try:
            self.tf_buffer.lookup_transform('odom', 'base_link', Time())
            checks['tf_odom_base_link'] = {'ok': True}
        except Exception as exc:  # noqa: BLE001
            checks['tf_odom_base_link'] = {'ok': False, 'error': str(exc)}
        now = time.monotonic()
        odom_ok = self.odom is not None and now - self.odom_wall < 2.0
        vx = self.odom.twist.twist.linear.x if self.odom else None
        wz = self.odom.twist.twist.angular.z if self.odom else None
        checks['odom_fresh_stationary'] = {
            'ok': odom_ok and abs(vx) <= a.stationary_v
            and abs(wz) <= a.stationary_w, 'vx': vx, 'wz': wz}
        checks['scan_fresh'] = {
            'ok': self.scan_min is not None and now - self.scan_wall < 2.0,
            'min_range_m': self.scan_min}
        base_pubs = sorted(i.node_name for i in
                           self.get_publishers_info_by_topic(
                               '/diff_drive_controller/cmd_vel'))
        base_subs = sorted(i.node_name for i in
                           self.get_subscriptions_info_by_topic(
                               '/diff_drive_controller/cmd_vel'))
        smoothed_subs = sorted(i.node_name for i in
                               self.get_subscriptions_info_by_topic(
                                   '/cmd_vel_smoothed'))
        checks['collision_protection'] = {
            'ok': base_pubs == ['collision_monitor']
            and 'collision_monitor' in smoothed_subs and bool(base_subs),
            'base_cmd_publishers': base_pubs,
            'base_cmd_subscribers': base_subs,
            'cmd_vel_smoothed_subscribers': smoothed_subs}
        slam_pubs = sorted(i.node_name for i in
                           self.get_publishers_info_by_topic('/map'))
        checks['online_slam_map'] = {
            'ok': self.map_msg is not None and slam_pubs == ['slam_toolbox'],
            'map_publishers': slam_pubs}
        checks['global_costmap'] = {'ok': self.costmap_msg is not None}
        checks['ros_domain'] = {
            'ok': self.get_parameter('use_sim_time').value,
            'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0'),
            'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION', '')}
        return checks

    # ------------------------------------------------------- goal selection
    def segment_check(self, start, goal):
        a = self.args
        slam, cost = Grid(self.map_msg), Grid(self.costmap_msg)
        length = math.hypot(goal[0] - start[0], goal[1] - start[1])
        samples = max(2, int(math.ceil(length / (slam.res / 2.0))) + 1)
        worst_cost = 0
        for k in range(samples):
            r = k / (samples - 1)
            if r * length < a.start_skip_m:
                # The robot already occupies this area; moving out is allowed.
                continue
            px = start[0] + r * (goal[0] - start[0])
            py = start[1] + r * (goal[1] - start[1])
            occ = slam.worst_in_disc(px, py, a.clearance_m)
            if occ is None or occ > a.max_occupancy:
                return None
            c = cost.value(px, py)
            if c is None or c < 0:
                return None
            worst_cost = max(worst_cost, c)
        goal_cost = cost.value(goal[0], goal[1])
        if goal_cost is None or goal_cost < 0:
            return None
        if worst_cost > a.max_segment_cost or goal_cost > a.max_goal_cost:
            return None
        return {'segment_max_cost': worst_cost, 'goal_cost': goal_cost}

    def select_goal(self, start, previous_sign):
        a = self.args
        if a.goal is not None:
            goal = (a.goal[0], a.goal[1], math.radians(a.goal[2]))
            return goal, self.segment_check(start, goal), 'fixed'
        best = None
        for bearing_deg in range(-90, 91, 15):
            if abs(bearing_deg) < a.min_yaw_change_deg:
                continue
            for dist in (0.55, 0.5, 0.6):
                heading = wrap(start[2] + math.radians(bearing_deg))
                goal = (start[0] + dist * math.cos(heading),
                        start[1] + dist * math.sin(heading),
                        wrap(heading + math.radians(a.extra_yaw_deg)))
                check = self.segment_check(start, goal)
                if check is None:
                    continue
                # Prefer low cost, gentle turns and alternating turn sides so
                # successive goals do not drift towards one wall.
                score = (check['segment_max_cost'] + check['goal_cost']
                         + 0.2 * abs(bearing_deg) + 2.0 * abs(dist - 0.55) * 100
                         + (15 if previous_sign and
                            math.copysign(1, bearing_deg) == previous_sign
                            else 0))
                if best is None or score < best[0]:
                    best = (score, goal, check)
        if best is None:
            return None, None, 'auto'
        return best[1], best[2], 'auto'

    # ---------------------------------------------------------- execution
    def run_goal(self, executor, index, start, goal, check, mode):
        a = self.args
        out = os.path.join(a.output, f'goal_{index:02d}')
        os.makedirs(out, exist_ok=True)
        events_file = open(os.path.join(out, 'action_events.csv'), 'w',
                           newline='')
        vel_file = open(os.path.join(out, 'velocities.csv'), 'w', newline='')
        events = csv.writer(events_file)
        events.writerow(['wall_s', 'sim_s', 'goal_uuid', 'event', 'map_x',
                         'map_y', 'yaw_deg', 'distance_remaining',
                         'navigation_s', 'recoveries', 'detail'])
        vel = csv.writer(vel_file)
        vel.writerow(['wall_s', 'sim_s', 'header_s', 'phase', 'topic',
                      'linear_x', 'angular_z'])
        wall_start = time.monotonic()
        rec = self.recording = {
            'uuid': '', 'phase': 'pre', 'wall_start': wall_start,
            'vel_csv': vel, 'vel_stats': {}, 'scan_min': math.inf,
            'collision_monitor_actions': [], 'stall_s': 0.0,
            'status_topic': None, 'last_feedback': None}

        def event(name, detail='', fb=None):
            row = [f'{time.monotonic() - wall_start:.4f}',
                   f'{self.sim_now():.4f}', rec['uuid'], name]
            if fb is not None:
                p = fb.current_pose.pose
                row += [f'{p.position.x:.4f}', f'{p.position.y:.4f}',
                        f'{math.degrees(yaw_of(p.orientation)):.3f}',
                        f'{fb.distance_remaining:.4f}',
                        f'{stamp_s(fb.navigation_time):.3f}',
                        fb.number_of_recoveries, detail]
            else:
                row += ['', '', '', '', '', '', detail]
            events.writerow(row)
            events_file.flush()
            vel_file.flush()

        state = {'handle': None, 'response': None, 'result': None,
                 'cancel_reason': None, 'cancel_response': None,
                 'cancel_sent_wall': None}

        def on_feedback(msg):
            rec['uuid'] = rec['uuid'] or uuid_hex(msg.goal_id)
            rec['last_feedback'] = msg.feedback
            event('feedback', fb=msg.feedback)

        def on_response(future):
            try:
                handle = future.result()
            except Exception as exc:  # noqa: BLE001
                state['response'] = 'error'
                event('send_error', str(exc))
                return
            state['response'] = 'accepted' if handle.accepted else 'rejected'
            if not handle.accepted:
                event('rejected')
                return
            state['handle'] = handle
            rec['uuid'] = uuid_hex(handle.goal_id)
            rec['accept_wall'] = time.monotonic()
            rec['accept_sim'] = self.sim_now()
            rec['phase'] = 'goal'
            event('accepted')
            handle.get_result_async().add_done_callback(on_result)
            if state['cancel_reason'] is not None:
                send_cancel()

        def on_result(future):
            try:
                wrapped = future.result()
                state['result'] = wrapped.status
            except Exception as exc:  # noqa: BLE001
                state['result'] = -1
                event('result_error', str(exc))
                return
            rec['terminal_wall'] = time.monotonic()
            rec['terminal_sim'] = self.sim_now()
            event('result', STATUS_NAMES.get(wrapped.status,
                                             str(wrapped.status)))

        def on_cancel_response(future):
            try:
                resp = future.result()
                state['cancel_response'] = {
                    'return_code': int(resp.return_code),
                    'goals_canceling': [uuid_hex(g.goal_id)
                                        for g in resp.goals_canceling]}
            except Exception as exc:  # noqa: BLE001
                state['cancel_response'] = {'error': str(exc)}
            event('cancel_response', json.dumps(state['cancel_response']))

        def send_cancel():
            state['cancel_sent_wall'] = time.monotonic()
            state['handle'].cancel_goal_async().add_done_callback(
                on_cancel_response)

        def cancel(reason):
            if state['cancel_reason'] is not None:
                return
            state['cancel_reason'] = reason
            state['cancel_sent_wall'] = time.monotonic()
            event('cancel_requested', reason)
            if state['handle'] is not None:
                send_cancel()

        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(goal[0])
        msg.pose.pose.position.y = float(goal[1])
        msg.pose.pose.orientation.z = math.sin(goal[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(goal[2] / 2.0)
        event('send', json.dumps({'goal': goal, 'start': start}))
        send_wall = time.monotonic()
        self.client.send_goal_async(
            msg, feedback_callback=on_feedback).add_done_callback(on_response)

        last_clock = self.sim_now()
        last_clock_wall = time.monotonic()
        interrupted = False
        try:
            while state['result'] is None:
                executor.spin_once(timeout_sec=0.05)
                now = time.monotonic()
                clock = self.sim_now()
                if clock < last_clock:
                    cancel('CLOCK_RESET')
                elif clock > last_clock:
                    last_clock, last_clock_wall = clock, now
                if state['response'] in ('rejected', 'error'):
                    break
                if state['response'] is None:
                    if now - send_wall > a.accept_timeout_s:
                        cancel('ACCEPT_TIMEOUT')
                        break
                    continue
                if state['cancel_reason'] is None:
                    if now - last_clock_wall > a.clock_stale_s:
                        cancel('CLOCK_STALE')
                    elif clock - rec['accept_sim'] > a.sim_timeout_s:
                        cancel('SIM_TIMEOUT')
                    elif now - rec['accept_wall'] > a.wall_guard_s:
                        cancel('WALL_GUARD')
                elif now - state['cancel_sent_wall'] > a.cancel_wait_s:
                    event('cancel_unconfirmed')
                    break
        except KeyboardInterrupt:
            interrupted = True
            cancel('USER_INTERRUPT')
            deadline = time.monotonic() + a.cancel_wait_s
            while state['result'] is None and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)

        if state['response'] is None:
            # A late acceptance is canceled by on_response; wait for it so no
            # orphan goal is left executing.
            self.spin_for(executor, 5.0, lambda: state['response'] is not None)
            if state['handle'] is not None:
                self.spin_for(executor, a.cancel_wait_s,
                              lambda: state['result'] is not None)

        rec['phase'] = 'settle'
        settle = self.settle(executor)
        final = None
        try:
            fx, fy, fyaw, fstamp = self.pose()
            final = {'x': fx, 'y': fy, 'yaw_deg': math.degrees(fyaw),
                     'tf_stamp_s': fstamp}
        except Exception as exc:  # noqa: BLE001
            final = {'error': str(exc)}
        self.spin_for(executor, 1.0)
        active_after = self.active_goals()
        rec['phase'] = 'done'
        event('done')
        events_file.close()
        vel_file.close()

        status = state['result']
        summary = {
            'index': index, 'mode': mode, 'goal_uuid': rec['uuid'],
            'start': {'x': start[0], 'y': start[1],
                      'yaw_deg': math.degrees(start[2])},
            'goal': {'x': goal[0], 'y': goal[1],
                     'yaw_deg': math.degrees(goal[2]),
                     'distance_m': math.hypot(goal[0] - start[0],
                                              goal[1] - start[1]),
                     'yaw_change_deg': math.degrees(wrap(goal[2] - start[2]))},
            'goal_check': check,
            'terminal_status': STATUS_NAMES.get(status, status),
            'status_topic_final': STATUS_NAMES.get(rec['status_topic'],
                                                   rec['status_topic']),
            'cancel_reason': state['cancel_reason'],
            'cancel_response': state['cancel_response'],
            'interrupted': interrupted,
            'final_pose': final, 'settle': settle,
            'active_goals_after': active_after,
            'velocity_stats_during_goal': rec['vel_stats'],
            'scan_min_during_goal_m': (None if rec['scan_min'] == math.inf
                                       else rec['scan_min']),
            'collision_monitor_actions': rec['collision_monitor_actions'],
            'commanded_stall_sim_s': rec['stall_s'],
        }
        if rec.get('terminal_wall') is not None:
            summary['duration_wall_s'] = rec['terminal_wall'] - rec['accept_wall']
            summary['duration_sim_s'] = rec['terminal_sim'] - rec['accept_sim']
        if rec['last_feedback'] is not None:
            fb = rec['last_feedback']
            summary['last_feedback'] = {
                'distance_remaining': fb.distance_remaining,
                'navigation_time_s': stamp_s(fb.navigation_time),
                'recoveries': fb.number_of_recoveries}
        if 'x' in final:
            xy = math.hypot(final['x'] - goal[0], final['y'] - goal[1])
            yaw_err = abs(wrap(math.radians(final['yaw_deg']) - goal[2]))
            summary['error_xy_m'] = xy
            summary['error_yaw_rad'] = yaw_err
        scan_min = summary['scan_min_during_goal_m']
        collision = ((scan_min is not None and scan_min < a.contact_range_m)
                     or rec['stall_s'] > 1.0)
        summary['collision_suspected'] = collision
        summary['acceptance'] = {
            'succeeded': status == GoalStatus.STATUS_SUCCEEDED,
            'xy_ok': summary.get('error_xy_m', math.inf) <= 0.10,
            'yaw_ok': summary.get('error_yaw_rad', math.inf) <= 0.15,
            'no_collision': not collision,
            'stationary': settle['stationary'],
            'no_active_goals_after': not active_after,
        }
        summary['pass'] = all(summary['acceptance'].values())
        with open(os.path.join(out, 'summary.json'), 'w') as fh:
            json.dump(summary, fh, indent=2, default=str)
        self.recording = None
        return summary, interrupted

    def settle(self, executor):
        """Wait until odom and base command are both at rest."""
        a = self.args
        deadline = time.monotonic() + a.settle_wall_s
        still_since = None
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if self.odom is None:
                continue
            vx = self.odom.twist.twist.linear.x
            wz = self.odom.twist.twist.angular.z
            base = self.last_cmd['/diff_drive_controller/cmd_vel']
            base_quiet = (base is None or time.monotonic() - base[0] > 0.6
                          or (abs(base[1]) < 1e-3 and abs(base[2]) < 1e-3))
            now_sim = self.sim_now()
            if abs(vx) <= a.stationary_v and abs(wz) <= a.stationary_w \
                    and base_quiet:
                still_since = still_since if still_since is not None else now_sim
                if now_sim - still_since >= a.stationary_s:
                    return {'stationary': True, 'odom_vx': vx, 'odom_wz': wz,
                            'still_sim_s': now_sim - still_since}
            else:
                still_since = None
        vx = self.odom.twist.twist.linear.x if self.odom else None
        wz = self.odom.twist.twist.angular.z if self.odom else None
        return {'stationary': False, 'odom_vx': vx, 'odom_wz': wz}


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('output')
    p.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'),
                   help='fixed map-frame goal (single run)')
    p.add_argument('--count', type=int, default=1,
                   help='consecutive auto-selected goals; stops at first fail')
    p.add_argument('--dry-run', action='store_true',
                   help='preflight and goal selection only; never sends')
    p.add_argument('--sim-timeout-s', type=float, default=120.0)
    p.add_argument('--wall-guard-s', type=float, default=600.0)
    p.add_argument('--clock-stale-s', type=float, default=15.0)
    p.add_argument('--accept-timeout-s', type=float, default=10.0)
    p.add_argument('--cancel-wait-s', type=float, default=30.0)
    p.add_argument('--settle-wall-s', type=float, default=30.0)
    p.add_argument('--stationary-s', type=float, default=1.0)
    p.add_argument('--stationary-v', type=float, default=0.01)
    p.add_argument('--stationary-w', type=float, default=0.02)
    p.add_argument('--clearance-m', type=float, default=0.30)
    p.add_argument('--max-occupancy', type=int, default=0)
    p.add_argument('--max-segment-cost', type=int, default=50)
    p.add_argument('--max-goal-cost', type=int, default=20)
    p.add_argument('--min-yaw-change-deg', type=float, default=20.0)
    p.add_argument('--extra-yaw-deg', type=float, default=0.0)
    p.add_argument('--contact-range-m', type=float, default=0.12)
    p.add_argument('--start-skip-m', type=float, default=0.15)
    args = p.parse_args(argv)
    if args.goal is not None:
        args.count = 1
    return args


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    os.makedirs(args.output, exist_ok=True)
    # rclpy's own handler would shut the context down before the goal can be
    # canceled, so interrupts are delivered as KeyboardInterrupt instead.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = Recorder(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    report = {'args': vars(args), 'runs': []}
    exit_code = 1
    try:
        previous_sign = 0
        for index in range(1, max(1, args.count) + 1):
            checks = node.preflight(executor)
            failed = [k for k, v in checks.items() if not v['ok']]
            run = {'index': index, 'preflight': checks}
            report['runs'].append(run)
            if failed:
                run['blocked'] = f'preflight failed: {failed}'
                break
            start = tuple(checks['tf_map_base_link']['pose'])
            goal, check, mode = node.select_goal(start, previous_sign)
            if goal is None or check is None:
                run['blocked'] = 'no goal passed live-map safety checks'
                break
            run['selected_goal'] = {'x': goal[0], 'y': goal[1],
                                    'yaw_deg': math.degrees(goal[2]),
                                    'check': check, 'mode': mode}
            if args.dry_run:
                exit_code = 0
                break
            summary, interrupted = node.run_goal(executor, index, start, goal,
                                                 check, mode)
            run['summary'] = summary
            node.get_logger().info(
                f'goal {index}: {summary["terminal_status"]} '
                f'xy={summary.get("error_xy_m")} '
                f'yaw={summary.get("error_yaw_rad")} pass={summary["pass"]}')
            if interrupted or not summary['pass']:
                break
            previous_sign = math.copysign(
                1, wrap(goal[2] - start[2])) if mode == 'auto' else 0
            if index == args.count:
                exit_code = 0
    finally:
        with open(os.path.join(args.output, 'report.json'), 'w') as fh:
            json.dump(report, fh, indent=2, default=str)
        node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
