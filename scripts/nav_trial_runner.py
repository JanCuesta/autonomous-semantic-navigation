#!/usr/bin/env python3
"""Supervised TurtleBot4 Nav2 trial runner (ROS 2 Jazzy).

DEFAULT IS PLANNING-ONLY. --execute explicitly enables one manually approved
NavigateToPose goal at a time. Never use on a table/dock. This does not replace
physical emergency stopping, command-ownership checks or lab approval.

Typical usage (laptop, ROS environment sourced):
  python3 nav_trial_runner.py --goals nav_test_goals.json
  python3 nav_trial_runner.py --goals nav_test_goals.json --execute
"""

import argparse
import csv
import json
import math
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def stamp():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def angle_error_deg(a, b):
    return (a - b + 180) % 360 - 180


def ros_stamp_age(msg):
    h = getattr(msg, 'header', None)
    if h is None:
        return None
    return time.time() - (h.stamp.sec + h.stamp.nanosec / 1e9)


def xy_pose(pose):
    return {'x': round(float(pose.position.x), 4),
            'y': round(float(pose.position.y), 4),
            'yaw_deg': round(yaw_deg(pose.orientation), 3)}


def load_goals(file):
    data = json.loads(file.read_text())
    goals = []
    for i, item in enumerate(data.get('goals', []), start=1):
        if not item.get('enabled', False):
            continue
        name = str(item.get('id', f'NAV{i:02d}'))
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,40}', name):
            raise ValueError(f'Invalid goal id: {name!r}')
        x, y, angle = (float(item[k]) for k in ('x', 'y', 'yaw_deg'))
        if not all(math.isfinite(x) for x in (x, y, angle)):
            raise ValueError(f'{name}: coordinates must be finite')
        goals.append({'id': name, 'x': x, 'y': y, 'yaw_deg': angle})
    if len({g['id'] for g in goals}) != len(goals):
        raise ValueError('Duplicate goal IDs')
    if not goals:
        raise ValueError('No enabled goals. Edit nav_test_goals.json using actual verified free-space map coordinates.')
    return goals


class TrialNode(Node):
    def __init__(self):
        super().__init__('asn_supervised_nav_trial_runner')
        self.last_scan = None
        self.last_odom = None
        self.last_amcl = None
        self.last_battery = None
        self.mission_state = None
        self.nav_status = None
        self.last_feedback = None
        self.goal_handle = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery, qos_profile_sensor_data)
        latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/semantic/mission/state', self.on_mission, latch)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_nav_status, latch)
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf, self)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.idle_client = self.create_client(Trigger, '/semantic/nbv/idle_check')

    def on_scan(self, msg): self.last_scan = msg
    def on_odom(self, msg): self.last_odom = msg
    def on_amcl(self, msg): self.last_amcl = msg
    def on_battery(self, msg): self.last_battery = msg
    def on_mission(self, msg): self.mission_state = msg.data
    def on_nav_status(self, msg): self.nav_status = msg
    def on_feedback(self, msg): self.last_feedback = msg.feedback

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.15, end - time.monotonic())))

    def wait(self, future, seconds, label):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            if future.done():
                return future.result()
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(f'{label} timed out after {seconds}s')

    def map_pose(self):
        try:
            transform = self.tf.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0))
            t, q = transform.transform.translation, transform.transform.rotation
            return {'x': round(t.x, 4), 'y': round(t.y, 4), 'yaw_deg': round(yaw_deg(q), 3)}
        except TransformException:
            return None

    def sample(self, elapsed):
        pose = self.map_pose()
        amcl = xy_pose(self.last_amcl.pose.pose) if self.last_amcl else None
        odom = xy_pose(self.last_odom.pose.pose) if self.last_odom else None
        f = self.last_feedback
        return {'utc': stamp(), 'elapsed_s': round(elapsed, 3), 'map_tf': pose,
                'amcl': amcl, 'odom': odom,
                'odom_vx': round(self.last_odom.twist.twist.linear.x, 4) if self.last_odom else None,
                'odom_wz': round(self.last_odom.twist.twist.angular.z, 4) if self.last_odom else None,
                'scan_age_s': round(ros_stamp_age(self.last_scan), 3) if self.last_scan else None,
                'odom_age_s': round(ros_stamp_age(self.last_odom), 3) if self.last_odom else None,
                'distance_remaining_m': round(float(f.distance_remaining), 4) if f is not None else None}

    def check_sensors(self):
        for name, msg in (('/scan', self.last_scan), ('/odom', self.last_odom)):
            age = ros_stamp_age(msg) if msg is not None else None
            if age is None or abs(age) >= 0.5:
                raise RuntimeError(f'{name} missing or not fresh (age {age})')
        if self.map_pose() is None:
            raise RuntimeError('map -> base_link TF unavailable; initialize REAL pose and verify map alignment')

    def check_battery(self):
        msg = self.last_battery
        age = ros_stamp_age(msg) if msg is not None else None
        if (msg is None or age is None or not math.isfinite(age)
                or abs(age) >= 10.0 or not math.isfinite(msg.percentage)
                or not 0.0 <= msg.percentage <= 1.0):
            raise RuntimeError('Battery reading unavailable, invalid or stale; do not execute navigation goals')
        if msg.percentage < 0.20:
            raise RuntimeError('Battery below 20%; do not execute navigation goals')

    def check_mission(self):
        if self.mission_state != 'IDLE':
            raise RuntimeError(f'Semantic mission must be IDLE (observed: {self.mission_state!r})')
        if self.nav_status and any(s.status in (1, 2, 3) for s in self.nav_status.status_list):
            raise RuntimeError('Another Nav2 NavigateToPose goal appears active')
        if not self.idle_client.wait_for_service(timeout_sec=3):
            raise RuntimeError('NBV idle-check service unavailable')
        res = self.wait(self.idle_client.call_async(Trigger.Request()), 4, 'NBV idle check')
        if not res.success:
            raise RuntimeError(f'NBV is not idle: {res.message}')

    def lifecycle_state(self, name):
        client = self.create_client(GetState, f'/{name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=2):
                return 'UNKNOWN (service unavailable)'
            result = self.wait(client.call_async(GetState.Request()), 3, f'{name} state')
            return result.current_state.label
        finally:
            self.destroy_client(client)

    @staticmethod
    def pose_msg(g):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = g['x'], g['y']
        a = math.radians(g['yaw_deg']) / 2
        msg.pose.orientation.z = math.sin(a)
        msg.pose.orientation.w = math.cos(a)
        return msg

    def make_plan(self, g, outdir):
        if not self.planner.wait_for_server(timeout_sec=8):
            raise RuntimeError('ComputePathToPose action server unavailable')
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose_msg(g)
        goal.use_start = False
        accepted = self.wait(self.planner.send_goal_async(goal), 10, 'planner acceptance')
        if not accepted or not accepted.accepted:
            raise RuntimeError('Planner rejected request')
        result = self.wait(accepted.get_result_async(), 30, 'planner result')
        code = getattr(result.result, 'error_code', None)
        path = result.result.path.poses
        with (outdir / 'plan.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw_deg'])
            for pose in path:
                p = xy_pose(pose.pose)
                writer.writerow([p['x'], p['y'], p['yaw_deg']])
        summary = {'status': int(result.status), 'error_code': int(code) if code is not None else None,
                   'pose_count': len(path)}
        (outdir / 'plan_result.json').write_text(json.dumps(summary, indent=2) + '\n')
        if result.status != 4 or code != 0 or len(path) < 2:
            raise RuntimeError(f'Planner did not produce a successful nonempty path: {summary}')
        return summary

    def execute(self, g, outdir, goal_timeout):
        if not self.navigator.wait_for_server(timeout_sec=8):
            raise RuntimeError('NavigateToPose action server unavailable')
        self.check_sensors()
        self.check_mission()
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_msg(g)
        self.last_feedback = None
        started = time.monotonic()
        start_pose = self.map_pose()
        accepted = self.wait(self.navigator.send_goal_async(goal, feedback_callback=self.on_feedback), 10,
                             'navigation acceptance')
        if not accepted or not accepted.accepted:
            raise RuntimeError('Navigation goal rejected')
        self.goal_handle = accepted
        future = accepted.get_result_async()
        status = None
        failure = None
        last_write = 0.0
        log = outdir / 'trajectory.csv'
        with log.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['utc', 'elapsed_s', 'tf_x', 'tf_y', 'tf_yaw_deg',
                'amcl_x', 'amcl_y', 'amcl_yaw_deg', 'odom_x', 'odom_y', 'odom_yaw_deg',
                'odom_vx', 'odom_wz', 'scan_age_s', 'odom_age_s', 'distance_remaining_m'])
            writer.writeheader()
            try:
                while not future.done():
                    elapsed = time.monotonic() - started
                    if elapsed >= goal_timeout:
                        failure = f'GOAL_TIMEOUT ({goal_timeout}s)'
                        break
                    rclpy.spin_once(self, timeout_sec=0.2)
                    elapsed = time.monotonic() - started
                    if elapsed - last_write >= 0.25:
                        s = self.sample(elapsed)
                        p, a, o = s['map_tf'] or {}, s['amcl'] or {}, s['odom'] or {}
                        writer.writerow({'utc': s['utc'], 'elapsed_s': s['elapsed_s'],
                            'tf_x': p.get('x'), 'tf_y': p.get('y'), 'tf_yaw_deg': p.get('yaw_deg'),
                            'amcl_x': a.get('x'), 'amcl_y': a.get('y'), 'amcl_yaw_deg': a.get('yaw_deg'),
                            'odom_x': o.get('x'), 'odom_y': o.get('y'), 'odom_yaw_deg': o.get('yaw_deg'),
                            'odom_vx': s['odom_vx'], 'odom_wz': s['odom_wz'],
                            'scan_age_s': s['scan_age_s'], 'odom_age_s': s['odom_age_s'],
                            'distance_remaining_m': s['distance_remaining_m']})
                        f.flush()
                        last_write = elapsed
                        if s['scan_age_s'] is None or s['odom_age_s'] is None or \
                                abs(s['scan_age_s']) >= 1.0 or abs(s['odom_age_s']) >= 1.0:
                            failure = 'SENSOR_STALE: requested cancel; physically stop robot if needed'
                            break
                        if s['map_tf'] is None:
                            failure = 'MAP_TF_MISSING: requested cancel; physically stop robot if needed'
                            break
                        if self.mission_state != 'IDLE':
                            failure = 'MISSION_NOT_IDLE: requested cancel; physically stop robot if needed'
                            break
                if failure:
                    self.cancel_active_goal()
                else:
                    status = int(future.result().status)
            except KeyboardInterrupt:
                failure = 'OPERATOR_INTERRUPTED: cancellation requested; verify physical stop'
                self.cancel_active_goal()
        self.spin_for(1.2)
        final_pose = self.map_pose()
        odom_age = ros_stamp_age(self.last_odom) if self.last_odom else None
        stopped_estimate = bool(self.last_odom and odom_age is not None and abs(odom_age) < 0.5
                         and abs(self.last_odom.twist.twist.linear.x) < 0.02
                         and abs(self.last_odom.twist.twist.angular.z) < 0.05)
        error = (math.hypot(final_pose['x'] - g['x'], final_pose['y'] - g['y'])
                 if final_pose else None)
        yaw_err = angle_error_deg(final_pose['yaw_deg'], g['yaw_deg']) if final_pose else None
        result = {'id': g['id'], 'requested_goal_map': g, 'start_map_tf': start_pose,
                  'final_map_tf': final_pose,
                  'final_amcl_estimate': xy_pose(self.last_amcl.pose.pose) if self.last_amcl else None,
                  'final_odom_estimate': xy_pose(self.last_odom.pose.pose) if self.last_odom else None,
                  'xy_error_m': round(error, 4) if error is not None else None,
                  'yaw_error_deg': round(yaw_err, 3) if yaw_err is not None else None,
                  'nav_status_code': status, 'nav_succeeded': status == 4 and failure is None,
                  'failure': failure, 'odom_stationary_snapshot': stopped_estimate,
                  'elapsed_s': round(time.monotonic() - started, 2), 'utc_finished': stamp(),
                  'physical_arrival_verified': 'NOT_RECORDED - operator must check',
                  'physical_stop_verified': 'NOT_RECORDED - operator must check'}
        (outdir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        self.goal_handle = None
        return result

    def cancel_active_goal(self):
        if self.goal_handle is None:
            return
        print('CANCEL REQUESTED. This is NOT an emergency stop. Verify wheels physically stopped!', flush=True)
        try:
            res = self.wait(self.goal_handle.cancel_goal_async(), 5, 'cancellation acknowledgement')
            print('Cancellation response:', res, flush=True)
        except Exception as ex:
            print('WARNING cancellation not confirmed:', ex, flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--goals', type=Path, required=True, help='JSON list of verified fixed map goals')
    ap.add_argument('--execute', action='store_true', help='MOTION ENABLED: require typed approval before each goal')
    ap.add_argument('--timeout', type=float, default=120.0, help='per-goal timeout in seconds (default 120)')
    ap.add_argument('--out', type=Path, default=Path('results/nav_trials'))
    args = ap.parse_args()
    if not (10 <= args.timeout <= 600):
        ap.error('--timeout must be 10-600 seconds')
    try:
        goals = load_goals(args.goals)
    except (OSError, KeyError, ValueError, TypeError, json.JSONDecodeError) as ex:
        ap.error(str(ex))
    rclpy.init()
    node = TrialNode()
    session = args.out / datetime.now().strftime('%Y%m%d_%H%M%S')
    session.mkdir(parents=True, exist_ok=False)
    failed = False
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            node.spin_for(0.25)
            if (node.last_scan is not None and node.last_odom is not None
                    and node.map_pose() is not None and node.mission_state is not None
                    and node.last_battery is not None):
                break
        print('Mission:', node.mission_state, '| battery:',
              round(node.last_battery.percentage * 100, 1) if node.last_battery else 'UNKNOWN', '%')
        print('Current map pose:', node.map_pose())
        if args.execute:
            print('\nMOTION MODE: use ONLY on the floor, with correct map alignment and independent stop ready.')
            for name in ('amcl', 'planner_server', 'controller_server', 'bt_navigator', 'collision_monitor'):
                state = node.lifecycle_state(name)
                print(name, state)
                if state != 'active':
                    raise RuntimeError(f'{name} not active: {state}')
            node.check_sensors()
            node.check_mission()
            node.check_battery()
        for g in goals:
            outdir = session / g['id']
            outdir.mkdir()
            (outdir / 'goal.json').write_text(json.dumps(g, indent=2) + '\n')
            print(f'\n=== {g["id"]}: map goal ({g["x"]:.2f}, {g["y"]:.2f}, {g["yaw_deg"]:.1f} deg) ===')
            node.check_sensors()
            plan = node.make_plan(g, outdir)
            print('PLAN SUCCESS:', plan['pose_count'], 'poses. Saved:', outdir / 'plan.csv')
            if not args.execute:
                print('PLAN-ONLY mode: no movement commanded.')
                continue
            print('Inspect the path and local/global costmaps in RViz; verify physical clearance and independent stop.')
            print('Confirm robot on floor, physically safe route, correct localization, battery and lab permission.')
            try:
                typed = input(f'Type MOVE {g["id"]} to start this one physical trial (anything else = stop): ').strip()
            except EOFError:
                typed = ''
            if typed != f'MOVE {g["id"]}':
                print('No navigation command sent. Stopping session.')
                break
            # Refresh callbacks after the operator has inspected the path.
            node.spin_for(1.0)
            node.check_sensors()
            node.check_mission()
            node.check_battery()
            result = node.execute(g, outdir, args.timeout)
            print('Nav2 success:', result['nav_succeeded'], '| estimated XY error (m):',
                  result['xy_error_m'], '| yaw error (deg):', result['yaw_error_deg'])
            print('Odometry stationary snapshot:', result['odom_stationary_snapshot'])
            print('Verify PHYSICAL stop, final position and heading; inspect', outdir / 'result.json')
            print('A stationary odometry snapshot is not proof of physically safe stopping.')
            if not result['nav_succeeded'] or not result['odom_stationary_snapshot']:
                print('TRIAL FAILED/UNCONFIRMED. DO NOT PROCEED until diagnosed and physically safe.')
                failed = True
                break
            try:
                verdict = input('Has the robot physically stopped, arrived safely, and is it approved to continue? Type YES: ').strip()
            except EOFError:
                verdict = ''
            result['physical_stop_verified'] = 'YES_BY_OPERATOR' if verdict == 'YES' else 'NOT_CONFIRMED'
            result['physical_arrival_verified'] = 'YES_BY_OPERATOR' if verdict == 'YES' else 'NOT_CONFIRMED'
            result['operator_confirmed_utc'] = stamp() if verdict == 'YES' else None
            (outdir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            if verdict != 'YES':
                print('Operator did not confirm; no further goals will be sent.')
                failed = True
                break
        print('\nSession results directory:', session.resolve())
    except KeyboardInterrupt:
        print('\nInterrupted. No more goals. If moving, verify physical stop independently!', flush=True)
        node.cancel_active_goal()
        failed = True
    except (RuntimeError, TimeoutError, Exception) as exc:
        print('STOP / BLOCKED:', exc, flush=True)
        if args.execute:
            print('DO NOT CONTINUE OR ASSUME STOPPED. Check robot and independent physical stop.', flush=True)
        node.cancel_active_goal()
        failed = True
    finally:
        results = []
        for f in sorted(session.glob('*/result.json')):
            try:
                results.append(json.loads(f.read_text()))
            except (ValueError, OSError):
                pass
        if results:
            with (session / 'summary.csv').open('w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['id', 'goal_x', 'goal_y', 'goal_yaw_deg',
                  'start_x', 'start_y', 'final_x', 'final_y', 'final_yaw_deg',
                  'xy_error_m', 'yaw_error_deg', 'nav_status_code', 'nav_succeeded',
                  'odom_stationary_snapshot', 'physical_stop_verified', 'physical_arrival_verified',
                  'elapsed_s', 'failure'])
                writer.writeheader()
                for r in results:
                    g, initial, final = r['requested_goal_map'], r.get('start_map_tf') or {}, r.get('final_map_tf') or {}
                    writer.writerow({'id': r['id'], 'goal_x': g['x'], 'goal_y': g['y'],
                        'goal_yaw_deg': g['yaw_deg'], 'start_x': initial.get('x'), 'start_y': initial.get('y'),
                        'final_x': final.get('x'), 'final_y': final.get('y'), 'final_yaw_deg': final.get('yaw_deg'),
                        'xy_error_m': r['xy_error_m'], 'yaw_error_deg': r['yaw_error_deg'],
                        'nav_status_code': r['nav_status_code'], 'nav_succeeded': r['nav_succeeded'],
                        'odom_stationary_snapshot': r['odom_stationary_snapshot'],
                        'physical_stop_verified': r['physical_stop_verified'],
                        'physical_arrival_verified': r['physical_arrival_verified'],
                        'elapsed_s': r['elapsed_s'], 'failure': r['failure']})
            print('Trial summary saved:', session / 'summary.csv')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
