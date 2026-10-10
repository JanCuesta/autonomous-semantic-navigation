#!/usr/bin/env python3
"""Step 9 short ROS 2 motion regression driver (Isaac already running).

Only publishes TwistStamped to /diff_drive_controller/cmd_vel, when --execute
is explicitly requested. Never launches/restarts Isaac or changes parameters.
Run with the passive sensor_timing_recorder in another terminal.

Safety: requires fresh simulated clock, fresh LiDAR and odometry, an isolated
command topic, active controllers, and >=0.50 m initial nearest valid LiDAR
range; stops when that clearance is violated or on any detected error.
This software guard is not a certified safety system. Verify open space visually.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

TOPIC = '/diff_drive_controller/cmd_vel'
PLAN = [
    ('baseline', 2.0, 0.0, 0.0),
    ('short_forward', 1.5, 0.10, 0.0),
    ('straight_settle', 3.0, 0.0, 0.0),
    ('left_turn', 2.0, 0.0, 0.40),
    ('turn_settle', 3.0, 0.0, 0.0),
    ('final_idle', 2.0, 0.0, 0.0),
]


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Explicitly authorize the planned robot movement.')
    parser.add_argument('--out', type=Path, default=None, help='Directory for commands.csv and motion_driver_report.json')
    parser.add_argument('--min-clearance', type=float, default=0.50,
                        help='Minimum nearest valid LiDAR return in metres, default 0.50.')
    parser.add_argument('--max-wall', type=float, default=120.,
                        help='Total wall-clock failsafe, default 120 seconds.')
    args = parser.parse_args()
    print('Planned motion, simulation seconds:')
    for name, duration, v, w in PLAN:
        print(f'  {name}: {duration:.1f}s  v={v:+.2f}m/s  w={w:+.2f}rad/s')
    print('Preflight requires an unobstructed robot and at least '
          f'{args.min_clearance:.2f} m nearest valid LiDAR clearance.')
    print('This driver does NOT provide independent Isaac ground truth.')
    if not args.execute:
        print('DRY RUN ONLY. Add --execute to authorize any motion.')
        return 0
    if args.min_clearance < 0.45 or args.max_wall < 15:
        parser.error('Minimum clearance must be >= 0.45 m; max-wall must be >= 15 s.')

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

    output = args.out or Path('results/validation/Step9') / (
        'motion_driver_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    if output.exists() and any(output.iterdir()):
        parser.error('Output exists and is not empty: '+str(output))
    rclpy.init()
    node = rclpy.create_node('asn_step9_motion_driver')
    state = dict(clock=None, clock_wall=None, scan=None, scan_stamp=None,
                 scan_wall=None, odom_stamp=None, odom_wall=None, error=None)
    commands = []
    phase_results = []
    started_wall = time.monotonic()
    pub = None
    result = 'incomplete'
    cause = None

    def on_clock(msg):
        t = seconds(msg.clock)
        if state['clock'] is not None and t < state['clock'] - 1e-6:
            state['error'] = 'Simulation time moved backwards.'
        if t != state['clock']:
            state['clock_wall'] = time.monotonic()
        state['clock'] = t

    def on_scan(msg):
        valid = [float(x) for x in msg.ranges if math.isfinite(x) and
                 msg.range_min <= x <= msg.range_max]
        state['scan'] = min(valid) if valid else None
        state['scan_stamp'] = seconds(msg.header.stamp)
        state['scan_wall'] = time.monotonic()

    def on_odom(msg):
        state['odom_stamp'] = seconds(msg.header.stamp)
        state['odom_wall'] = time.monotonic()

    def pump(duration=0.015):
        rclpy.spin_once(node, timeout_sec=duration)

    def check_health(moving=False):
        now = time.monotonic()
        if state['error']:
            raise RuntimeError(state['error'])
        if now - started_wall > args.max_wall:
            raise RuntimeError('Total wall-clock failsafe exceeded.')
        if state['clock'] is None or state['clock_wall'] is None or now-state['clock_wall'] > 2.0:
            raise RuntimeError('Clock unavailable or stopped for >2 wall seconds.')
        if state['scan_wall'] is None or now-state['scan_wall'] > 2.0:
            raise RuntimeError('LiDAR samples missing for >2 wall seconds.')
        if state['odom_wall'] is None or now-state['odom_wall'] > 2.0:
            raise RuntimeError('Odometry samples missing for >2 wall seconds.')
        for topic, stamp in (('scan', state['scan_stamp']), ('odom', state['odom_stamp'])):
            if stamp is None or state['clock']-stamp > .25:
                raise RuntimeError(topic+' data older than 0.25 simulation seconds.')
        if state['scan'] is None:
            raise RuntimeError('No finite valid LiDAR ranges.')
        if moving and state['scan'] < args.min_clearance:
            raise RuntimeError(f'LiDAR clearance {state["scan"]:.3f}m below '
                               f'{args.min_clearance:.3f}m stop threshold.')
        if pub is not None and node.count_publishers(TOPIC) != 1:
            raise RuntimeError('Unexpected concurrent command publisher detected.')

    def send(v=0.0, w=0.0):
        if pub is None or state['clock'] is None:
            return
        msg = TwistStamped()
        sim = state['clock']
        msg.header.stamp.sec = int(sim)
        msg.header.stamp.nanosec = int(round((sim-int(sim))*1e9))
        if msg.header.stamp.nanosec >= 1_000_000_000:
            msg.header.stamp.sec += 1
            msg.header.stamp.nanosec = 0
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        pub.publish(msg)
        commands.append({'wall_monotonic_s': time.monotonic(), 'sim_time_s': sim,
                         'v_mps': v, 'w_rad_s': w})

    try:
        node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(Odometry, '/odom', on_odom, reliable)
        service = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not service.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Controller manager service unavailable.')
        fut = service.call_async(ListControllers.Request())
        limit = time.monotonic()+6
        while not fut.done() and time.monotonic()<limit:
            pump()
        if not fut.done() or fut.result() is None:
            raise RuntimeError('Controller state query failed.')
        active = {c.name: c.state for c in fut.result().controller}
        for name in ('joint_state_broadcaster','diff_drive_controller'):
            if active.get(name) != 'active':
                raise RuntimeError(name+' not active: '+str(active))
        if node.count_publishers(TOPIC) != 0:
            raise RuntimeError('Command topic has another publisher; aborting.')
        ready_deadline = time.monotonic() + 8
        while (state['clock'] is None or state['scan_wall'] is None or
               state['odom_wall'] is None) and time.monotonic()<ready_deadline:
            pump()
        check_health(moving=True)
        if node.count_subscribers(TOPIC) != 1:
            raise RuntimeError('Expected exactly one controller command subscriber.')
        pub = node.create_publisher(TwistStamped, TOPIC, 10)
        t = time.monotonic()+4
        while pub.get_subscription_count() < 1 and time.monotonic()<t:
            pump()
        if pub.get_subscription_count() != 1:
            raise RuntimeError('Controller command subscriber did not match.')
        print(f'Preflight OK. Closest valid LiDAR range: {state["scan"]:.3f} m.',flush=True)
        # A short zero-command preamble allows the controller to confirm receipt.
        next_send = 0.
        for name, duration, v, w in PLAN:
            start = state['clock']
            end = start+duration
            wall_deadline = time.monotonic()+max(15.,duration*7.)
            print(f'Phase: {name} ({duration:.1f} sim s)',flush=True)
            while state['clock'] < end:
                pump(.004)
                check_health(moving=(v != 0. or w != 0.))
                now = time.monotonic()
                if now > wall_deadline:
                    raise RuntimeError('Phase wall-time deadline exceeded: '+name)
                if now >= next_send:
                    send(v,w)
                    next_send = now + .02
            send(0.,0.)
            phase_results.append(dict(phase=name, start_sim_s=start,end_sim_s=state['clock']))
        result='completed'
    except (Exception, KeyboardInterrupt) as exc:
        cause=str(exc) or exc.__class__.__name__
        print('ABORTED: '+cause, flush=True)
    finally:
        # Always request zero before exiting; controller has its own 0.5-sim-s timeout.
        if pub is not None:
            until = time.monotonic()+1.0
            while time.monotonic()<until:
                send(0.,0.)
                pump(.025)
        output.mkdir(parents=True,exist_ok=True)
        with (output/'commands.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=['wall_monotonic_s','sim_time_s','v_mps','w_rad_s'])
            writer.writeheader()
            writer.writerows(commands)
        (output/'motion_driver_report.json').write_text(json.dumps({
            'status':result,'error':cause,'phases':phase_results,'commands':len(commands),
            'last_scan_clearance_m':state['scan'],
            'calibrated_odometry_parameters_unchanged':True,
            'note':'No independent Isaac ground truth recorded; evaluate alongside passive recorder.'
        },indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
        print(f'Driver status: {result}; output: {output}',flush=True)
    return 0 if result=='completed' else 1


if __name__=='__main__':
    raise SystemExit(main())
