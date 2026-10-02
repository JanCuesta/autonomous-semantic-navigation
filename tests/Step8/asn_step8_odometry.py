"""Step 8 wheel-odometry validation harness.

Purpose:
- Compare position-based diff_drive_controller odometry against independent
  Isaac articulation-root ground truth for /World/V1_Robot/base_link.
- Keep Isaac ground truth evaluation-only; it is never published into the
  navigation TF tree.
- Use simulation time for all trial durations.
- Preserve wheel-position evidence, but do not use the deferred raw wheel
  velocity-integral issue as a Step 8 acceptance criterion.

Usage:
1. Keep Isaac Sim open with Testing_World+Robot.usd and press Play.
2. Ensure joint_state_broadcaster and diff_drive_controller are active.
3. Run this file in the Isaac Script Editor to arm the passive recorder.
4. In a ROS-sourced terminal, run for example:
       python3 asn_step8_odometry.py --ros --scenario straight
5. Repeat by re-arming the Isaac side for each scenario.

Supported scenarios:
    straight, reverse, left_turn, right_turn, square

Results:
    ~/projects/autonomous-semantic-navigation/results/step8/<scenario>_<UTC timestamp>/

No physics, gains, controller configuration, robot pose, or TF ownership is changed.
"""

import csv
import json
import math
import sys
import time
from pathlib import Path

SESSION = Path('/tmp/asn_step8_odometry_session.json')
WHEELS = ('left_wheel_joint', 'right_wheel_joint')
CMD = '/diff_drive_controller/cmd_vel'
ODOM = '/diff_drive_controller/odom'

RADIUS = 0.03575
SEPARATION = 0.233


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


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_xyzw(qx, qy, qz, qw):
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    )


def pose_delta(rows, start, end):
    points = [r for r in rows if start <= r['t'] <= end]
    if len(points) < 2:
        return {'error': 'Insufficient pose samples', 'samples': len(points)}

    first, last = points[0], points[-1]
    dx = last['x'] - first['x']
    dy = last['y'] - first['y']
    yaw0 = first['yaw']

    # Express displacement in each source's own start-body frame.
    # This lets Isaac world coordinates and odom coordinates have different
    # global origins/orientations without corrupting the relative comparison.
    forward = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    lateral = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    dyaw = wrap_angle(last['yaw'] - first['yaw'])

    return {
        'samples': len(points),
        'covered_s': last['t'] - first['t'],
        'start': {'x_m': first['x'], 'y_m': first['y'], 'yaw_rad': first['yaw']},
        'end': {'x_m': last['x'], 'y_m': last['y'], 'yaw_rad': last['yaw']},
        'dx_global_m': dx,
        'dy_global_m': dy,
        'distance_m': math.hypot(dx, dy),
        'forward_m': forward,
        'lateral_m': lateral,
        'yaw_change_rad': dyaw,
        'yaw_change_deg': math.degrees(dyaw),
    }


def wheel_position_delta(rows, start, end):
    points = [r for r in rows if start <= r['t'] <= end]
    if len(points) < 2:
        return {'error': 'Insufficient joint-position samples', 'samples': len(points)}

    first, last = points[0], points[-1]
    dq_l = last['left_q'] - first['left_q']
    dq_r = last['right_q'] - first['right_q']
    ds_l = RADIUS * dq_l
    ds_r = RADIUS * dq_r

    return {
        'samples': len(points),
        'covered_s': last['t'] - first['t'],
        'left_angle_change_rad': dq_l,
        'right_angle_change_rad': dq_r,
        'left_distance_m': ds_l,
        'right_distance_m': ds_r,
        'forward_m': 0.5 * (ds_l + ds_r),
        'yaw_change_rad': (ds_r - ds_l) / SEPARATION,
        'yaw_change_deg': math.degrees((ds_r - ds_l) / SEPARATION),
    }


def comparison(gt, odom, wheels):
    if any('error' in value for value in (gt, odom, wheels)):
        return {'error': 'One or more source metrics were unavailable.'}

    heading_error = wrap_angle(odom['yaw_change_rad'] - gt['yaw_change_rad'])
    wheel_heading_error = wrap_angle(wheels['yaw_change_rad'] - gt['yaw_change_rad'])

    return {
        'odom_minus_ground_truth': {
            'forward_error_m': odom['forward_m'] - gt['forward_m'],
            'lateral_error_m': odom['lateral_m'] - gt['lateral_m'],
            'translation_vector_error_m': math.hypot(
                odom['forward_m'] - gt['forward_m'],
                odom['lateral_m'] - gt['lateral_m']
            ),
            'heading_error_rad': heading_error,
            'heading_error_deg': math.degrees(heading_error),
        },
        'wheel_positions_minus_ground_truth': {
            'forward_error_m': wheels['forward_m'] - gt['forward_m'],
            'heading_error_rad': wheel_heading_error,
            'heading_error_deg': math.degrees(wheel_heading_error),
        },
        'odom_minus_wheel_positions': {
            'forward_error_m': odom['forward_m'] - wheels['forward_m'],
            'heading_error_rad': wrap_angle(
                odom['yaw_change_rad'] - wheels['yaw_change_rad']
            ),
            'heading_error_deg': math.degrees(wrap_angle(
                odom['yaw_change_rad'] - wheels['yaw_change_rad']
            )),
        },
    }


def scenario_sequence(name):
    # Conservative nominal commands. A settle phase is included so the
    # evaluated trial captures controller acceleration/braking dynamics.
    basic = {
        'straight': [
            ('BASELINE', 1.0, 0.0, 0.0),
            ('FORWARD', 2.0, 0.05, 0.0),
            ('SETTLE', 2.0, 0.0, 0.0),
        ],
        'reverse': [
            ('BASELINE', 1.0, 0.0, 0.0),
            ('REVERSE', 2.0, -0.05, 0.0),
            ('SETTLE', 2.0, 0.0, 0.0),
        ],
        'left_turn': [
            ('BASELINE', 1.0, 0.0, 0.0),
            ('LEFT_TURN', 3.0, 0.0, 0.30),
            ('SETTLE', 2.0, 0.0, 0.0),
        ],
        'right_turn': [
            ('BASELINE', 1.0, 0.0, 0.0),
            ('RIGHT_TURN', 3.0, 0.0, -0.30),
            ('SETTLE', 2.0, 0.0, 0.0),
        ],
    }
    if name in basic:
        return basic[name]

    if name == 'square':
        seq = [('BASELINE', 1.0, 0.0, 0.0)]
        for side in range(1, 5):
            seq.append((f'SIDE_{side}', 3.0, 0.05, 0.0))
            seq.append((f'SIDE_{side}_SETTLE', 0.8, 0.0, 0.0))
            seq.append((f'TURN_{side}', 4.0, 0.0, 0.40))
            seq.append((f'TURN_{side}_SETTLE', 0.8, 0.0, 0.0))
        seq.append(('FINAL_SETTLE', 2.0, 0.0, 0.0))
        return seq

    raise ValueError(
        f'Unknown scenario {name!r}. '
        'Choose straight, reverse, left_turn, right_turn, or square.'
    )


def requested_scenario():
    if '--scenario' not in sys.argv:
        return 'straight'
    i = sys.argv.index('--scenario')
    if i + 1 >= len(sys.argv):
        raise RuntimeError('--scenario requires a value.')
    return sys.argv[i + 1]


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
    assert math.isclose(
        UsdGeom.GetStageMetersPerUnit(context.get_stage()),
        1.0,
        abs_tol=1e-6
    ), 'This recording expects the existing metre-scale stage.'

    view = tensors.create_simulation_view('warp', stage_id=context.get_stage_id())
    robot = view.create_articulation_view('/World/V1_Robot')
    assert robot.count == 1, f'Expected one robot, found {robot.count}'

    links = list(robot.shared_metatype.link_names)
    assert links and links[0] == 'base_link', (
        f'Expected articulation root link base_link, got {links!r}'
    )

    names = list(robot.shared_metatype.dof_names)
    indices = [names.index(name) for name in WHEELS]

    # Validate the readers before arming.
    for getter in (
        robot.get_dof_positions,
        robot.get_dof_velocities,
        robot.get_dof_velocity_targets,
        robot.get_root_transforms,
    ):
        getter().numpy()

    folder = Path(tempfile.mkdtemp(prefix='asn_step8_'))
    save_json(SESSION, {'folder': str(folder), 'expires': time.time() + 300})

    async def record(simulation_view=view):
        recent, rows = deque(maxlen=512), []
        active, previous, heartbeat = False, None, 0.0
        armed_deadline, active_deadline = time.monotonic() + 300, None
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
                    active = True
                    active_deadline = now + 180
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

                qx, qy, qz, qw = [float(value) for value in pose[3:7]]
                row = {
                    't': t,
                    'x': float(pose[0]),
                    'y': float(pose[1]),
                    'z': float(pose[2]),
                    'qx': qx,
                    'qy': qy,
                    'qz': qz,
                    'qw': qw,
                    'yaw': yaw_from_xyzw(qx, qy, qz, qw),
                }

                for side, index in zip(('left', 'right'), indices):
                    row.update({
                        side + '_q': float(q[index]),
                        # Retained only as raw evidence from Step 7.
                        # Step 8 does not use velocity integrals for acceptance.
                        side + '_v': float(v[index]),
                        side + '_target': float(target[index]),
                    })

                (rows if active else recent).append(row)

        except Exception as exc:
            status, error = 'error', str(exc)

        finally:
            save_csv(folder / 'isaac.csv', rows)
            save_json(folder / 'finished.json', {'status': status, 'error': error})
            print(f'ISAAC recording finished: {status}; samples={len(rows)}')
            if error:
                print(error)

    globals()['asn_step8_recording_task'] = asyncio.ensure_future(record())

    print('STEP 8 ISAAC RECORDER ARMED (300 seconds).')
    print('Root ground truth: /World/V1_Robot/base_link')
    print('Passive recorder only; no pose, target, gain, physics, or TF changes.')


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

    scenario = requested_scenario()
    sequence = scenario_sequence(scenario)

    if not SESSION.exists():
        raise RuntimeError('Run this file in Isaac Script Editor first.')

    session = json.loads(SESSION.read_text())
    folder = Path(session['folder'])
    if time.time() > session['expires'] or (folder / 'finished.json').exists():
        raise RuntimeError('Isaac recorder expired or finished. Arm it again.')

    project = Path.home() / 'projects/autonomous-semantic-navigation'
    assert project.is_dir(), f'Project directory not found: {project}'

    output = (
        project / 'results/step8' /
        (scenario + '_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    )
    output.mkdir(parents=True)

    joints, odom, commands, phases = [], [], [], []
    state = {
        't': None,
        'stamp': None,
        'advanced': time.monotonic(),
        'error': None,
    }

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('asn_step8_odometry_' + str(int(time.time())))
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
            if i >= len(msg.position):
                state['error'] = 'Wheel position missing from JointState.'
                return
            row[side + '_q'] = float(msg.position[i])
            if i < len(msg.velocity):
                row[side + '_v'] = float(msg.velocity[i])

        if not all(math.isfinite(value) for value in row.values()):
            state['error'] = 'Non-finite joint feedback.'
        joints.append(row)

    def on_odom(msg):
        q = msg.pose.pose.orientation
        yaw = yaw_from_xyzw(q.x, q.y, q.z, q.w)
        odom.append({
            't': seconds(msg.header.stamp),
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'qx': float(q.x),
            'qy': float(q.y),
            'qz': float(q.z),
            'qw': float(q.w),
            'yaw': yaw,
            'v': float(msg.twist.twist.linear.x),
            'omega': float(msg.twist.twist.angular.z),
        })

    def publish(linear, angular):
        msg = TwistStamped()
        msg.header.stamp = state['stamp']
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        publisher.publish(msg)
        commands.append({
            't': state['t'],
            'requested_linear_mps': linear,
            'requested_angular_rad_s': angular,
        })

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
        node.create_subscription(Odometry, ODOM, on_odom, qos_profile_sensor_data)

        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Controller manager unavailable.')

        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('Controller status query timed out.')

        controllers = {c.name: c.state for c in future.result().controller}
        required = ('diff_drive_controller', 'joint_state_broadcaster')
        if any(controllers.get(name) != 'active' for name in required):
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

        if node.count_publishers('/clock') != 1:
            raise RuntimeError('Expected exactly one /clock publisher.')
        if node.count_publishers('/joint_states') != 1:
            raise RuntimeError('Expected exactly one /joint_states publisher.')
        if node.count_publishers(ODOM) != 1:
            raise RuntimeError('Expected exactly one odometry publisher.')

        check_health()

        publisher = node.create_publisher(TwistStamped, CMD, 10)
        deadline = time.monotonic() + 3
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError('Movement controller has no matching command subscription.')

        (folder / 'run').write_text('start')
        started = True

        for label, duration, linear, angular in sequence:
            start = state['t']
            end = start + duration
            next_send, next_graph_check = 0.0, 0.0
            wall_deadline = time.monotonic() + max(20.0, duration * 8.0)

            print(
                f'{label}: {duration:.1f} simulation seconds; '
                f'linear={linear:+.3f} m/s; angular={angular:+.3f} rad/s',
                flush=True
            )

            while state['t'] < end:
                rclpy.spin_once(node, timeout_sec=0.01)
                check_health()
                now = time.monotonic()

                if now > wall_deadline:
                    raise RuntimeError('Phase exceeded its wall-time limit.')

                if now >= next_graph_check:
                    others = [
                        p.node_name
                        for p in node.get_publishers_info_by_topic(CMD)
                        if p.node_name != node.get_name()
                    ]
                    if others:
                        raise RuntimeError(f'Competing command publisher: {others}')
                    next_graph_check = now + 0.5

                if state['t'] >= end:
                    break

                if now >= next_send:
                    publish(linear, angular)
                    next_send = now + 0.02

            publish(0.0, 0.0)
            phases.append({
                'name': label,
                'start': start,
                'end': state['t'],
                'requested_linear_mps': linear,
                'requested_angular_rad_s': angular,
            })

    except BaseException as exc:
        failure = 'Interrupted' if isinstance(exc, KeyboardInterrupt) else str(exc)
        print('ABORTED: ' + failure, flush=True)

    finally:
        if publisher is not None and state['stamp'] is not None:
            try:
                until = time.monotonic() + 0.5
                while time.monotonic() < until:
                    publish(0.0, 0.0)
                    rclpy.spin_once(node, timeout_sec=0.02)
            except BaseException as exc:
                print(
                    'Explicit stop publication interrupted: '
                    f'{exc}; controller timeout remains configured.'
                )

        if started:
            (folder / 'stop').write_text('stop')

        save_csv(output / 'ros_joint_states.csv', joints)
        save_csv(output / 'ros_odometry.csv', odom)
        save_csv(output / 'commands.csv', commands)

        node.destroy_node()
        rclpy.shutdown()

    isaac = []
    if started:
        deadline = time.monotonic() + 5
        while not (folder / 'finished.json').exists() and time.monotonic() < deadline:
            time.sleep(0.05)

        raw = folder / 'isaac.csv'
        if raw.exists():
            shutil.copy2(raw, output / 'isaac.csv')
            with raw.open(newline='') as stream:
                isaac = [
                    {k: float(v) for k, v in row.items()}
                    for row in csv.DictReader(stream)
                ]

        finished = folder / 'finished.json'
        if finished.exists():
            recorder = json.loads(finished.read_text())
            if recorder['status'] != 'ok':
                failure = failure or recorder['error']
        else:
            failure = failure or 'Isaac recorder did not finalize.'

        if not isaac:
            failure = failure or 'No raw Isaac samples were collected.'

    report = {
        'status': 'aborted' if failure else 'recorded',
        'error': failure,
        'scenario': scenario,
        'wheel_radius_m': RADIUS,
        'wheel_separation_m': SEPARATION,
        'ground_truth_source': '/World/V1_Robot articulation root = base_link',
        'odometry_source': ODOM,
        'odometry_mode': 'position feedback, open_loop=false',
        'raw_wheel_velocity_issue': (
            'Deferred from Step 7; raw velocity integrals are not used '
            'for Step 8 acceptance.'
        ),
        'phases': phases,
    }

    if phases and isaac and odom and joints:
        # Exclude BASELINE from trial evaluation. Include final settling so
        # acceleration/braking effects belong to the measured physical result.
        trial_start = phases[1]['start'] if len(phases) > 1 else phases[0]['start']
        trial_end = phases[-1]['end']

        gt = pose_delta(isaac, trial_start, trial_end)
        od = pose_delta(odom, trial_start, trial_end)
        wh = wheel_position_delta(joints, trial_start, trial_end)

        report['trial_window'] = {'start': trial_start, 'end': trial_end}
        report['isaac_ground_truth'] = gt
        report['ros_odometry'] = od
        report['wheel_position_kinematics'] = wh
        report['comparison'] = comparison(gt, od, wh)

        print('\n===== STEP 8 NOMINAL COMPARISON =====')
        print('Scenario:', scenario)
        print('Isaac ground truth:')
        print(json.dumps(gt, indent=2))
        print('\nROS wheel odometry:')
        print(json.dumps(od, indent=2))
        print('\nWheel-position kinematics:')
        print(json.dumps(wh, indent=2))
        print('\nErrors:')
        print(json.dumps(report['comparison'], indent=2))

    save_json(output / 'report.json', report)

    print(f'\nStatus: {report["status"]}')
    print(f'Results: {output}')
    print(
        'Recorded != accepted. Review ground-truth/odometry translation, '
        'heading, repeatability, and limitations before marking Step 8 complete.'
    )

    if failure:
        raise SystemExit(1)


if '--ros' in sys.argv:
    run_ros()
else:
    start_isaac()
