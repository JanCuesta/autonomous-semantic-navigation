#!/usr/bin/env python3
"""V1 non-invasive synchronized timing/sensor/TF evidence recorder.

Captures /clock, /scan, /joint_states, /odom, /tf, /tf_static on a common
wall-clock timeline, preserving message simulation-time stamps. This tool
never publishes commands, launches Isaac Sim, or edits robot parameters.

Use one phase at a time with --phase stationary|motion|slam_load. The SAME
recorder and measurement schema are reused for all three phases. The motion
phase must be commanded externally; this recorder cannot move the robot.
"""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import threading
import time
from collections import Counter, defaultdict

TOPICS = ('/clock', '/scan', '/joint_states', '/odom', '/tf', '/tf_static')


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def finite(x):
    return isinstance(x, (float, int)) and math.isfinite(x)


def safe_stats(values):
    vals = sorted(v for v in values if finite(v))
    if not vals:
        return None
    return {'count': len(vals), 'min': vals[0], 'median': statistics.median(vals),
            'p95': vals[math.ceil(0.95 * len(vals)) - 1],
            'max': vals[-1], 'mean': statistics.mean(vals)}


def rate_report(rows):
    """Use message simulation stamps, not wall-clock arrival frequency."""
    stamps = [r['stamp_sim_s'] for r in rows if finite(r.get('stamp_sim_s'))]
    if len(stamps) < 2:
        return {'sim_hz': None, 'count': len(stamps), 'note': 'insufficient samples'}
    diffs = [b - a for a, b in zip(stamps, stamps[1:])]
    pos = [d for d in diffs if d > 1e-9]
    span = stamps[-1] - stamps[0]
    return {
        'sim_hz': (len(pos) / span) if span > 0 and pos else None,
        'count': len(stamps),
        'unique_timestamp_transitions': len(pos),
        'duplicate_stamps': sum(abs(d) <= 1e-9 for d in diffs),
        'backwards_stamps': sum(d < -1e-9 for d in diffs),
        'largest_gap_sim_s': max(pos) if pos else None,
        'positive_period_sim_s': safe_stats(pos),
        'age_at_arrival_sim_s': safe_stats([r.get('age_sim_s') for r in rows]),
    }


def rtf_report(clock_rows, window_wall_s=5.0):
    if len(clock_rows) < 2:
        return {'mean': None, 'windows': [], 'note': 'insufficient clock samples'}
    x0, x1 = clock_rows[0], clock_rows[-1]
    d_wall = x1['wall_s'] - x0['wall_s']
    d_sim = x1['stamp_sim_s'] - x0['stamp_sim_s']
    windows = []
    bucket_start = 0
    for idx in range(1, len(clock_rows)):
        a = clock_rows[bucket_start]
        b = clock_rows[idx]
        elapsed = b['wall_s'] - a['wall_s']
        if elapsed >= window_wall_s:
            windows.append({'wall_start_s': a['wall_s'], 'wall_end_s': b['wall_s'],
                            'sim_start_s': a['stamp_sim_s'], 'sim_end_s': b['stamp_sim_s'],
                            'rtf': (b['stamp_sim_s'] - a['stamp_sim_s']) / elapsed})
            bucket_start = idx
    values = [w['rtf'] for w in windows]
    return {'mean': d_sim / d_wall if d_wall > 0 else None,
            'windows': windows, 'window_rtf_stats': safe_stats(values),
            'windows_under_0_40': sum(v < 0.4 for v in values),
            'clock_backwards': sum(b['stamp_sim_s'] < a['stamp_sim_s'] - 1e-9
                                   for a, b in zip(clock_rows, clock_rows[1:])),
            'clock_zero_advance_arrivals': sum(abs(b['stamp_sim_s']-a['stamp_sim_s']) <= 1e-9
                                               for a,b in zip(clock_rows,clock_rows[1:])),
            'largest_clock_arrival_gap_wall_s': max(
                (b['wall_s']-a['wall_s'] for a,b in zip(clock_rows,clock_rows[1:])), default=None)}


def read_file(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, PermissionError):
        return None


def machine_info():
    try:
        p = subprocess.run(['powerprofilesctl', 'get'], capture_output=True, text=True,
                           timeout=3, check=False)
        profile = p.stdout.strip() if p.returncode == 0 else 'unavailable'
    except (OSError, subprocess.TimeoutExpired):
        profile = 'unavailable'
    ac = {p.parent.name: read_file(p) for p in Path('/sys/class/power_supply').glob('*/online')}
    return {
        'power_profile': profile, 'ac_online': ac,
        'cpu_boost': read_file('/sys/devices/system/cpu/cpufreq/boost'),
        'hostname': os.uname().nodename,
    }



def resources_snapshot():
    sample = {'wall_monotonic_s': time.monotonic(),
              'load_average': list(os.getloadavg()), 'thermal_zones': {}}
    for zone in Path('/sys/class/thermal').glob('thermal_zone*'):
        typ = read_file(zone / 'type')
        raw = read_file(zone / 'temp')
        if typ and raw:
            try:
                temp = float(raw) / 1000.0
                if -30 <= temp <= 150:
                    sample['thermal_zones'][zone.name + ':' + typ] = temp
            except ValueError:
                pass
    try:
        cmd = ['nvidia-smi', '--query-gpu=temperature.gpu,utilization.gpu,memory.used,power.draw',
               '--format=csv,noheader,nounits']
        p = subprocess.run(cmd, text=True, capture_output=True, check=False, timeout=3)
        if p.returncode == 0:
            sample['gpu'] = p.stdout.strip().splitlines()
        else:
            sample['gpu'] = 'unavailable'
    except (OSError, subprocess.TimeoutExpired):
        sample['gpu'] = 'unavailable'
    return sample


def collect_resources_until(stop, recorder, interval_s=10):
    while not stop.is_set():
        try:
            recorder.resource_rows.append(resources_snapshot())
        except Exception as e:
            recorder.errors.append('Optional resource snapshot error: '+str(e))
        stop.wait(interval_s)


def save_csv(path, rows):
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, 'w', newline='') as f:
        if keys:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)


class Recorder:
    def __init__(self, node):
        self.node = node
        self.wall_zero = time.monotonic()
        self.clock = None
        self.clock_wall = None
        self.clock_rows = []
        self.rows = defaultdict(list)
        self.scan_rows = []
        self.odom_rows = []
        self.joint_rows = []
        self.tf_rows = []
        self.errors = []
        self.resource_rows = []
        self.clock_resets = 0
        self.publishers = {}

    def wall(self):
        return time.monotonic() - self.wall_zero

    def event(self, topic, stamp=None, frame=''):
        wall = self.wall()
        age = None if stamp is None or self.clock is None else self.clock - stamp
        r = {'wall_s': wall, 'stamp_sim_s': stamp, 'clock_at_arrival_sim_s': self.clock,
             'age_sim_s': age, 'frame_id': frame}
        self.rows[topic].append(r)
        return r

    def on_clock(self, msg):
        t = seconds(msg.clock)
        if self.clock is not None and t < self.clock - 1e-9:
            self.clock_resets += 1
        if self.clock is None or abs(t-self.clock) > 1e-9:
            self.clock_wall = time.monotonic()  # last time simulation actually advanced
        self.clock = t
        r = self.event('/clock', t)
        self.clock_rows.append(r)

    def on_scan(self, msg):
        r = self.event('/scan', seconds(msg.header.stamp), msg.header.frame_id)
        valid = [float(x) for x in msg.ranges if finite(x) and msg.range_min <= x <= msg.range_max]
        self.scan_rows.append({**r, 'beam_count': len(msg.ranges), 'valid_beams': len(valid),
                               'closest_valid_m': min(valid) if valid else None,
                               'returns_below_0_3m': sum(x < .3 for x in valid),
                               'returns_below_1m': sum(x < 1. for x in valid),
                               'range_min_m': msg.range_min, 'range_max_m': msg.range_max,
                               'scan_time_s': msg.scan_time,
                               'time_increment_s': msg.time_increment})

    def on_joint(self, msg):
        r = self.event('/joint_states', seconds(msg.header.stamp), msg.header.frame_id)
        q = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity))
        self.joint_rows.append({**r, 'left_wheel_rad': q.get('left_wheel_joint'),
                                'right_wheel_rad': q.get('right_wheel_joint'),
                                'left_raw_rad_s': vel.get('left_wheel_joint'),
                                'right_raw_rad_s': vel.get('right_wheel_joint')})

    def on_odom(self, msg):
        r = self.event('/odom', seconds(msg.header.stamp), msg.header.frame_id)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.odom_rows.append({**r, 'child_frame_id': msg.child_frame_id,
                               'x_m': p.x, 'y_m': p.y, 'yaw_rad': yaw,
                               'vx_mps': msg.twist.twist.linear.x,
                               'wz_rad_s': msg.twist.twist.angular.z})

    def on_tf(self, msg, topic):
        for t in msg.transforms:
            r = self.event(topic, None if topic == '/tf_static' else seconds(t.header.stamp), t.header.frame_id)
            self.tf_rows.append({**r, 'topic': topic, 'child_frame_id': t.child_frame_id,
                                 'translation_x_m': t.transform.translation.x,
                                 'translation_y_m': t.transform.translation.y,
                                 'translation_z_m': t.transform.translation.z})

    def snapshot_publishers(self):
        for t in TOPICS:
            try:
                endpoints = self.node.get_publishers_info_by_topic(t)
                self.publishers[t] = sorted([f'{x.node_namespace}/{x.node_name}' for x in endpoints])
            except Exception as e:
                self.publishers[t] = [f'ERROR: {e}']

    def summary(self, phase, clock_target_s, wall_budget_s):
        timing = rtf_report(self.clock_rows)
        rates = {t: rate_report(self.rows[t]) for t in TOPICS}
        tf_pairs = Counter((r['topic'], r['frame_id'], r['child_frame_id']) for r in self.tf_rows)
        scan_frames = Counter(r['frame_id'] for r in self.scan_rows)
        closest = safe_stats([r['closest_valid_m'] for r in self.scan_rows])
        beams = sorted(set(r['beam_count'] for r in self.scan_rows))
        ranges_min = sorted(set(round(r['range_min_m'], 6) for r in self.scan_rows))
        ranges_max = sorted(set(round(r['range_max_m'], 6) for r in self.scan_rows))
        odom_frames = sorted(set((r['frame_id'],r['child_frame_id']) for r in self.odom_rows))
        delta_pose = None
        if len(self.odom_rows) > 1:
            a,b = self.odom_rows[0], self.odom_rows[-1]
            delta_pose = {'displacement_m': math.hypot(b['x_m']-a['x_m'],b['y_m']-a['y_m']),
                          'delta_yaw_rad_wrapped': math.atan2(math.sin(b['yaw_rad']-a['yaw_rad']),
                                                              math.cos(b['yaw_rad']-a['yaw_rad']))}
        observed_tf = [{'topic': k[0], 'parent': k[1], 'child': k[2], 'sample_count': v}
                       for k,v in sorted(tf_pairs.items())]
        flags = []
        if not self.clock_rows: flags.append('No /clock samples')
        if self.clock_resets: flags.append(f'/clock moved backwards {self.clock_resets} times')
        for t in ('/scan', '/joint_states', '/odom', '/tf', '/tf_static'):
            if not self.rows[t]: flags.append('No samples: '+t)
        if '/scan' in rates and rates['/scan']['sim_hz'] is not None and rates['/scan']['sim_hz'] < 9:
            flags.append('LiDAR below provisional 9 simulated Hz')
        if '/odom' in rates and rates['/odom']['sim_hz'] is not None and rates['/odom']['sim_hz'] < 27:
            flags.append('Odometry below provisional 27 simulated Hz')
        if not any(r['frame_id']=='odom' and r['child_frame_id']=='base_link' and
                   r['topic']=='/tf' for r in self.tf_rows):
            flags.append('No dynamic odom -> base_link observed')
        if not any(r['frame_id']=='base_link' and r['child_frame_id']=='laser_frame' and
                   r['topic']=='/tf_static' for r in self.tf_rows):
            flags.append('No static base_link -> laser_frame observed')
        if len(self.clock_rows)>1 and self.clock_rows[-1]['stamp_sim_s']-self.clock_rows[0]['stamp_sim_s'] < .9*clock_target_s:
            flags.append('Insufficient simulated-time coverage')
        return {'phase': phase, 'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
                'simulation_duration_requested_s': clock_target_s, 'wall_budget_s': wall_budget_s,
                'power': machine_info(), 'publishers': self.publishers,
                'resources': self.resource_rows, 'timing': timing, 'rates_sim_time': rates,
                'scan': {'frames': dict(scan_frames), 'beam_counts': beams,
                         'range_min_m': ranges_min, 'range_max_m': ranges_max,
                         'closest_valid_m': closest},
                'odom': {'frame_pairs': odom_frames, 'start_to_end': delta_pose},
                'tf_observed_pairs': observed_tf,
                'warnings': flags, 'errors': self.errors,
                'interpretation_note': 'Observed TF pairs identify the frame edges on /tf, not the exact publishing process. '
                  'Publisher endpoint names are listed separately. Frequencies use simulation stamps; '
                  'RTF uses wall and simulation time. No independent Isaac ground truth is captured.'}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--phase', choices=['stationary','motion','slam_load'], default='stationary')
    ap.add_argument('--duration-sim', type=float, default=60., help='Length in simulated seconds')
    ap.add_argument('--max-wall', type=float, default=240., help='Failsafe elapsed wall seconds')
    ap.add_argument('--out', type=Path, default=None, help='Output directory (default: /tmp/asn_sensor_timing_<phase>_<UTC>)')
    args = ap.parse_args()
    if args.duration_sim <= 1 or args.max_wall <= 5:
        ap.error('duration-sim must exceed 1 and max-wall must exceed 5')
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan, JointState
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
    except ImportError as e:
        ap.error('Requires ROS 2 Jazzy environment sourced: '+str(e))
    out = args.out or Path('/tmp')/('asn_sensor_timing_'+args.phase+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    if out.exists() and any(out.iterdir()):
        ap.error('Output directory already exists and is non-empty: '+str(out))
    rclpy.init()
    node = rclpy.create_node('asn_sensor_timing_recorder')
    recorder = Recorder(node)
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # Keep subscription objects alive in the node until shutdown.
    node.create_subscription(Clock, '/clock', recorder.on_clock, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', recorder.on_scan, qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', recorder.on_joint, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', recorder.on_odom, reliable)
    node.create_subscription(TFMessage, '/tf', lambda msg: recorder.on_tf(msg, '/tf'), qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf_static', lambda msg: recorder.on_tf(msg, '/tf_static'), static_qos)
    print('Passive synchronized recorder: '+args.phase, flush=True)
    print('NO commands will be published. NO simulator changes will be made.', flush=True)
    resource_stop = threading.Event()
    resource_thread = threading.Thread(target=collect_resources_until,
                                       args=(resource_stop, recorder), daemon=True)
    resource_thread.start()
    wall_start = time.monotonic()
    sim_start = None
    last_progress = wall_start
    last_display = wall_start
    try:
        while time.monotonic() - wall_start < args.max_wall:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if recorder.clock is not None:
                if sim_start is None:
                    sim_start = recorder.clock
                if recorder.clock >= sim_start + args.duration_sim:
                    break
                if recorder.clock_wall is not None and recorder.clock_wall > last_progress:
                    last_progress = recorder.clock_wall
            if now - last_display > 5:
                prog = None if sim_start is None or recorder.clock is None else recorder.clock-sim_start
                print(f'Elapsed: {now-wall_start:.1f} wall s, '+
                      (f'{prog:.1f}/{args.duration_sim:.1f} sim s' if prog is not None else 'waiting for /clock')+
                      f', scans: {len(recorder.scan_rows)}, odom: {len(recorder.odom_rows)}', flush=True)
                last_display = now
            if sim_start is not None and now - last_progress > 5:
                recorder.errors.append('No /clock progress for >5 wall seconds')
                break
        else:
            recorder.errors.append('Wall-time budget reached before target simulated duration')
    except KeyboardInterrupt:
        recorder.errors.append('Interrupted before planned completion')
    finally:
        resource_stop.set()
        resource_thread.join(timeout=4)
        recorder.snapshot_publishers()
        out.mkdir(parents=True, exist_ok=True)
        save_csv(out/'clock.csv', recorder.clock_rows)
        save_csv(out/'events.csv', [{**r, 'topic':t} for t,rows in recorder.rows.items() for r in rows])
        save_csv(out/'scan_summary.csv', recorder.scan_rows)
        save_csv(out/'joint_states.csv', recorder.joint_rows)
        save_csv(out/'odom.csv', recorder.odom_rows)
        save_csv(out/'tf_pairs.csv', recorder.tf_rows)
        (out/'resources.json').write_text(json.dumps(recorder.resource_rows, indent=2, allow_nan=False)+'\n')
        report = recorder.summary(args.phase, args.duration_sim, args.max_wall)
        (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        node.destroy_node()
        rclpy.shutdown()
        print('Saved: '+str(out), flush=True)
        print('Warnings: '+str(len(report['warnings']))+'; errors: '+str(len(report['errors'])), flush=True)
        for w in report['warnings'] + report['errors']:
            print(' - '+w, flush=True)
        print('Sim RTF: '+str(report['timing']['mean']), flush=True)
        for t in ('/scan','/joint_states','/odom'):
            print(t+' sim Hz: '+str(report['rates_sim_time'][t]['sim_hz']), flush=True)


if __name__ == '__main__':
    main()
