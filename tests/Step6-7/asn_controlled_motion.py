"""One controlled Isaac/ROS recording; no physics or controller configuration edits.

1. Run this file in the already-open Isaac Script Editor (passive recorder).
2. In a ROS-sourced Ubuntu terminal: python3 asn_controlled_motion.py --ros

Command: zero for 1 sim second, +0.05 m/s for 1 sim second, zero for 2 sim seconds.
Requires the existing active diff_drive_controller and joint_state_broadcaster.
Results: ~/projects/autonomous-semantic-navigation/results/step7/motion_<timestamp>/
The two processes coordinate through /tmp on the SAME Ubuntu computer.
"""

import csv
import json
import math
import sys
import time
from pathlib import Path

SESSION = Path('/tmp/asn_controlled_motion_session.json')
WHEELS = ('left_wheel_joint', 'right_wheel_joint')
CMD = '/diff_drive_controller/cmd_vel'
RADIUS = 0.03575  # Existing project YAML; not a configuration change.


def save_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2))
    temporary.replace(path)


def save_csv(path, rows):
    if not rows:
        return
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def joint_metrics(rows, prefix, start, end):
    # Use actual covered sample timestamps; do not pretend endpoints were sampled.
    points = []
    for row in rows:
        if not start <= row['t'] <= end:
            continue
        if points and row['t'] < points[-1]['t']:
            return {'samples': len(points), 'error': 'Sample timestamps went backwards'}
        if not points or row['t'] > points[-1]['t']:
            points.append(row)
    if len(points) < 2:
        return {'samples': len(points), 'error': 'Insufficient samples'}
    q, v = prefix + '_q', prefix + '_v'
    integral = sum((b['t'] - a['t']) * (a[v] + b[v]) / 2
                   for a, b in zip(points, points[1:]))
    delta = points[-1][q] - points[0][q]
    return dict(samples=len(points), covered_s=points[-1]['t'] - points[0]['t'],
                angle_change_rad=delta, velocity_integral_rad=integral,
                residual_rad=delta - integral,
                velocity_min=min(r[v] for r in points),
                velocity_max=max(r[v] for r in points))


def displacement(rows, start, end):
    points = [r for r in rows if start <= r['t'] <= end]
    if len(points) < 2:
        return {'error': 'Insufficient pose samples'}
    dx, dy = (points[-1][k] - points[0][k] for k in ('x', 'y'))
    return dict(dx_m=dx, dy_m=dy, distance_m=math.hypot(dx, dy),
                covered_s=points[-1]['t'] - points[0]['t'])


def start_isaac():
    import asyncio
    import tempfile
    from collections import deque
    import omni.kit.app
    import omni.timeline
    import omni.usd
    import omni.physics.tensors as tensors
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import UsdGeom

    context = omni.usd.get_context()
    assert omni.timeline.get_timeline_interface().is_playing(), 'Press Play first.'
    assert math.isclose(UsdGeom.GetStageMetersPerUnit(context.get_stage()), 1.0,
                        abs_tol=1e-6), 'This recording expects the existing metre-scale stage.'
    view = tensors.create_simulation_view('warp', stage_id=context.get_stage_id())
    robot = view.create_articulation_view('/World/V1_Robot')
    assert robot.count == 1, f'Expected one robot, found {robot.count}'
    names = list(robot.shared_metatype.dof_names)
    indices = [names.index(name) for name in WHEELS]
    folder = Path(tempfile.mkdtemp(prefix='asn_motion_'))
    save_json(SESSION, {'folder': str(folder), 'expires': time.time() + 180})

    async def record(simulation_view=view):
        recent, rows = deque(maxlen=512), []
        active, previous, heartbeat = False, None, 0.0
        armed_deadline, active_deadline = time.monotonic() + 180, None
        status, error = 'ok', None
        try:
            while True:
                await omni.kit.app.get_app().next_update_async()
                now = time.monotonic()
                if now - heartbeat >= 0.5:
                    (folder / 'heartbeat').write_text(str(time.time()))
                    heartbeat = now
                if (folder / 'stop').exists():
                    break
                if now > (active_deadline if active else armed_deadline):
                    raise RuntimeError('Recorder expired; run the file in Isaac again.')
                if not active and (folder / 'run').exists():
                    active, active_deadline = True, now + 60
                    rows.extend(recent)
                    recent.clear()
                t = float(SimulationManager.get_simulation_time())
                if previous is not None and t < previous:
                    raise RuntimeError('Simulation time reset during recording.')
                if previous is not None and t == previous:
                    continue
                previous = t
                q = robot.get_dof_positions().numpy()[0].copy()
                v = robot.get_dof_velocities().numpy()[0].copy()
                target = robot.get_dof_velocity_targets().numpy()[0].copy()
                pose = robot.get_root_transforms().numpy()[0].copy()
                row = dict(t=t, x=float(pose[0]), y=float(pose[1]), z=float(pose[2]))
                for side, index in zip(('left', 'right'), indices):
                    row.update({side + '_q': float(q[index]),
                                side + '_v': float(v[index]),
                                side + '_target': float(target[index])})
                (rows if active else recent).append(row)
        except Exception as exc:
            status, error = 'error', str(exc)
        finally:
            save_csv(folder / 'isaac.csv', rows)
            save_json(folder / 'finished.json', {'status': status, 'error': error})
            print(f'ISAAC recording finished: {status}; samples={len(rows)}')
            if error:
                print(error)

    # Retain task reference for Script Editor execution.
    globals()['asn_motion_recording_task'] = asyncio.ensure_future(record())
    print('ISAAC RECORDER ARMED (180 seconds). Now run the terminal command.')
    print('Passive recording only: no targets, gains, poses, or settings changed.')


def run_ros():
    import shutil
    import signal
    from datetime import datetime, timezone
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from controller_manager_msgs.srv import ListControllers
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState

    if not SESSION.exists():
        raise RuntimeError('Run this file in Isaac Script Editor first.')
    session = json.loads(SESSION.read_text())
    folder = Path(session['folder'])
    if time.time() > session['expires'] or (folder / 'finished.json').exists():
        raise RuntimeError('Isaac recorder expired or finished. Arm it again.')
    project = Path.home() / 'projects/autonomous-semantic-navigation'
    assert project.is_dir(), f'Project directory not found: {project}'
    output = project / 'results/step7' / ('motion_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    output.mkdir(parents=True)
    joints, odom, commands, phases = [], [], [], []
    state = {'t': None, 'stamp': None, 'advanced': time.monotonic(), 'error': None}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('asn_controlled_motion_' + str(int(time.time())))
    publisher, started, failure = None, False, None

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)

    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_clock(msg):
        t = seconds(msg.clock)
        if state['t'] is not None and t < state['t']:
            state['error'] = 'Simulation clock reset.'
        if t != state['t']:
            state['advanced'] = time.monotonic()
        state.update(t=t, stamp=msg.clock)

    def on_joint(msg):
        if not all(name in msg.name for name in WHEELS):
            return
        row = {'t': seconds(msg.header.stamp)}
        for side, name in zip(('left', 'right'), WHEELS):
            i = msg.name.index(name)
            if i >= len(msg.position) or i >= len(msg.velocity):
                state['error'] = 'Wheel position or velocity missing from JointState.'
                return
            row.update({side + '_q': float(msg.position[i]), side + '_v': float(msg.velocity[i])})
        if not all(math.isfinite(value) for value in row.values()):
            state['error'] = 'Non-finite joint feedback.'
        joints.append(row)

    def on_odom(msg):
        odom.append(dict(t=seconds(msg.header.stamp), x=msg.pose.pose.position.x,
                         y=msg.pose.pose.position.y, v=msg.twist.twist.linear.x,
                         omega=msg.twist.twist.angular.z))

    def publish(speed):
        msg = TwistStamped()
        msg.header.stamp = state['stamp']
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = speed
        publisher.publish(msg)
        commands.append(dict(t=state['t'], requested_mps=speed))

    def check_health():
        if state['error']:
            raise RuntimeError(state['error'])
        if time.monotonic() - state['advanced'] > 2:
            raise RuntimeError('Simulation clock stopped advancing.')
        heartbeat = folder / 'heartbeat'
        if not heartbeat.exists() or time.time() - heartbeat.stat().st_mtime > 3:
            raise RuntimeError('Isaac recorder is not running.')
        if (folder / 'finished.json').exists():
            raise RuntimeError('Isaac recorder ended before the sequence finished.')

    try:
        node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
        node.create_subscription(JointState, '/joint_states', on_joint, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/diff_drive_controller/odom', on_odom, qos_profile_sensor_data)
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Controller manager unavailable.')
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('Controller status query timed out.')
        controllers = {c.name: c.state for c in future.result().controller}
        if any(controllers.get(name) != 'active' for name in ('diff_drive_controller', 'joint_state_broadcaster')):
            raise RuntimeError(f'Required controllers are not active: {controllers}')
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if state['t'] is not None and len(joints) >= 2 and odom:
                break
        if state['t'] is None or len(joints) < 2 or not odom:
            raise RuntimeError('Clock, joint feedback, or odometry unavailable.')
        if abs(joints[-1]['t'] - state['t']) > 1 or abs(odom[-1]['t'] - state['t']) > 1:
            raise RuntimeError('Feedback timestamps do not match simulation time.')
        if node.count_publishers(CMD):
            raise RuntimeError('Another command publisher exists. Stop it before this recording.')
        if node.count_publishers('/clock') != 1 or node.count_publishers('/joint_states') != 1:
            raise RuntimeError('Expected exactly one clock and one joint-state publisher.')
        check_health()
        publisher = node.create_publisher(TwistStamped, CMD, 10)
        deadline = time.monotonic() + 3
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError('Movement controller has no matching command subscription.')
        (folder / 'run').write_text('start')
        started = True
        for label, duration, speed in (('BASELINE', 1.0, 0.0), ('MOVE', 1.0, 0.05), ('STOP', 2.0, 0.0)):
            start = state['t']
            end, next_send, next_graph_check = start + duration, 0.0, 0.0
            wall_deadline = time.monotonic() + 20
            print(f'{label}: {duration:.1f} simulation seconds; command={speed:.2f} m/s', flush=True)
            while state['t'] < end:
                rclpy.spin_once(node, timeout_sec=0.01)
                check_health()
                now = time.monotonic()
                if now > wall_deadline:
                    raise RuntimeError('Phase exceeded its wall-time limit.')
                if now >= next_graph_check:
                    others = [p.node_name for p in node.get_publishers_info_by_topic(CMD)
                              if p.node_name != node.get_name()]
                    if others:
                        raise RuntimeError(f'Competing command publisher: {others}')
                    next_graph_check = now + 0.5
                if state['t'] >= end:
                    break
                if now >= next_send:
                    publish(speed)
                    next_send = now + 0.02
            publish(0.0)
            phases.append(dict(name=label, start=start, end=state['t']))
    except BaseException as exc:
        failure = 'Interrupted' if isinstance(exc, KeyboardInterrupt) else str(exc)
        print('ABORTED: ' + failure, flush=True)
    finally:
        # Context stays alive on Ctrl+C, allowing explicit stop publication.
        if publisher is not None and state['stamp'] is not None:
            try:
                until = time.monotonic() + 0.5
                while time.monotonic() < until:
                    publish(0.0)
                    rclpy.spin_once(node, timeout_sec=0.02)
            except BaseException as exc:
                print(f'Explicit stop publication interrupted: {exc}; controller timeout remains configured.')
        if started:
            (folder / 'stop').write_text('stop')
        save_csv(output / 'ros_joint_states.csv', joints)
        save_csv(output / 'ros_odometry.csv', odom)
        save_csv(output / 'commands.csv', commands)
        node.destroy_node()
        rclpy.shutdown()

    isaac = []
    if started:
        deadline = time.monotonic() + 4
        while not (folder / 'finished.json').exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        raw = folder / 'isaac.csv'
        if raw.exists():
            shutil.copy2(raw, output / 'isaac.csv')
            with raw.open(newline='') as stream:
                isaac = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
        finished = folder / 'finished.json'
        if finished.exists():
            recorder = json.loads(finished.read_text())
            if recorder['status'] != 'ok':
                failure = failure or recorder['error']
        else:
            failure = failure or 'Isaac recorder did not finalize.'
        if not isaac:
            failure = failure or 'No raw Isaac samples were collected.'

    report = dict(status='aborted' if failure else 'recorded', error=failure,
                  wheel_radius_m=RADIUS, phases=[],
                  expected_steady_wheel_rad_s=0.05 / RADIUS)
    print('\nCOMPARISON (angles and integrals in radians):')
    for phase in phases:
        item = dict(phase)
        print('\n' + phase['name'])
        for source, rows in (('ROS', joints), ('ISAAC', isaac)):
            item[source] = {}
            for side in ('left', 'right'):
                m = joint_metrics(rows, side, phase['start'], phase['end'])
                item[source][side] = m
                if 'error' in m:
                    print(f'  {source} {side}: {m["error"]}')
                else:
                    print(f'  {source} {side}: covered={m["covered_s"]:.3f}s; '
                          f'angle={m["angle_change_rad"]:+.6f}; '
                          f'integral(v)={m["velocity_integral_rad"]:+.6f}; '
                          f'v_min/max={m["velocity_min"]:+.3f}/{m["velocity_max"]:+.3f}')
        target_rows = [r for r in isaac if phase['start'] <= r['t'] <= phase['end']]
        if target_rows:
            item['targets'] = {side: [min(r[side + '_target'] for r in target_rows),
                                     max(r[side + '_target'] for r in target_rows)]
                               for side in ('left', 'right')}
            print('  Isaac wheel target ranges (rad/s): ' + str(item['targets']))
        report['phases'].append(item)
    if len(phases) == 3:
        start, end = phases[1]['start'], phases[2]['end']
        report['actual_isaac_displacement'] = displacement(isaac, start, end)
        report['ros_odom_displacement'] = displacement(odom, start, end)
        report['last_stop_second_displacement'] = displacement(isaac, end - 1, end)
        for label in ('actual_isaac_displacement', 'ros_odom_displacement', 'last_stop_second_displacement'):
            print(f'\n{label}: {report[label]}')
        dq = [joint_metrics(joints, side, start, end) for side in ('left', 'right')]
        if all('angle_change_rad' in m for m in dq):
            report['wheel_angle_distance_m'] = RADIUS * sum(m['angle_change_rad'] for m in dq) / 2
            print(f'\nWheel-angle-derived forward distance: {report["wheel_angle_distance_m"]:.6f} m')
    save_json(output / 'report.json', report)
    print(f'\nStatus: {report["status"]}. Results: {output}')
    print('A completed recording is not automatically a validation pass; compare the measurements.')
    if failure:
        raise SystemExit(1)


if '--ros' in sys.argv:
    run_ros()
else:
    start_isaac()
