#!/usr/bin/env python3
"""Final-dynamics baseline: Isaac Script Editor recorder + ROS terminal driver.

No arguments (Isaac only): arm passive base_link recorder.
--ros: run an isolated simulation-time command campaign.
--plan / --self-test / --analyze DIR: offline, no ROS/Isaac imports.
See README.md. Historical tests/Step8 evidence and robot settings are untouched.
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
import statistics
import sys
import time
from pathlib import Path

SESSION = Path('/tmp/asn_turtlebot4_step8_session.json')
WHEELS = ('left_wheel_joint', 'right_wheel_joint')
CMD = '/cmd_vel'
ODOM = '/odom'
RADIUS = 0.03575
SEPARATION = 0.233
MAX_GAP = 0.20  # simulation seconds; rejected intervals, never extrapolated


def save_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def save_csv(path, rows):
    if rows:
        fields = list(dict.fromkeys(key for row in rows for key in row))
        with Path(path).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)


def read_csv(path):
    with Path(path).open(newline='') as stream:
        return [{k: float(v) for k, v in row.items() if v != ''}
                for row in csv.DictReader(stream)]


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_xyzw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*qy + qz*qz))


def sequence(scenario, repeats):
    phases = []

    def add(name, duration, v=0., w=0., kind='idle', trial=None, repeat=None):
        phases.append(dict(name=name, duration=duration, v=v, w=w, kind=kind,
                           trial=trial, repeat=repeat))

    def trial(name, duration, v, w, kind, repeat):
        add(name, duration, v, w, kind, name, repeat)
        add(name + '_settle', 2.0, trial=name, repeat=repeat)

    add('baseline', 1.5)
    for repeat in range(1, repeats + 1):
        suffix = f'_r{repeat}'
        if scenario == 'smoke':
            trial('smoke_forward' + suffix, 2., .10, 0., 'linear', repeat)
            trial('smoke_reverse' + suffix, 2., -.10, 0., 'linear', repeat)
        if scenario in ('linear', 'sweep', 'all'):
            # Forward/reverse pairs limit cumulative excursion. No pose resets.
            for speed in (.10, .25, .40):
                for sign, direction in ((1, 'forward'), (-1, 'reverse')):
                    trial(f'{direction}_{speed:.2f}' + suffix, 3.,
                          sign * speed, 0., 'linear', repeat)
        if scenario in ('angular', 'sweep', 'all'):
            for speed in (.40, .75, 1.10):
                for sign, direction in ((1, 'left'), (-1, 'right')):
                    trial(f'{direction}_{speed:.2f}' + suffix, 4.,
                          0., sign * speed, 'angular', repeat)
        if scenario in ('arc', 'all'):
            trial('arc_forward' + suffix, 4., .25, .75, 'arc', repeat)
            trial('arc_reverse' + suffix, 4., -.25, -.75, 'arc', repeat)
        if scenario in ('square', 'all'):
            # Feedforward durations include nominal ramp-up/braking area.
            # The real closure is measured, not enforced by GT feedback.
            side_t = .75/.25 + .5*.25*(1/.60 - 1/.80)
            turn_t = (math.pi/2)/.75 + .5*.75*(1/1.80 - 1/2.20)
            for side in range(1, 5):
                trial(f'square_side_{side}' + suffix, side_t, .25, 0.,
                      'square_linear', repeat)
                trial(f'square_turn_{side}' + suffix, turn_t, 0., .75,
                      'square_angular', repeat)
    add('final_settle', 2.)
    return phases


class Series:
    """Monotonic series with continuous yaw and bounded linear interpolation."""
    def __init__(self, rows):
        if len(rows) < 2:
            raise ValueError('Fewer than two samples.')
        self.rows = []
        yaw = None
        for source in rows:
            row = dict(source)
            if not all(math.isfinite(v) for v in row.values()):
                raise ValueError('Non-finite sample.')
            if self.rows and row['t'] < self.rows[-1]['t']:
                raise ValueError('Source timestamps moved backwards.')
            if self.rows and row['t'] == self.rows[-1]['t']:
                continue
            if 'yaw' in row:
                yaw = row['yaw'] if yaw is None else (
                    yaw + wrap_angle(row['yaw'] - self.rows[-1]['yaw']))
                row['yaw_continuous'] = yaw
            self.rows.append(row)
        if len(self.rows) < 2:
            raise ValueError('Fewer than two distinct timestamps.')
        self.times = [r['t'] for r in self.rows]

    def at(self, t):
        i = bisect.bisect_left(self.times, t)
        if i < len(self.times) and abs(self.times[i] - t) < 1e-8:
            return dict(self.rows[i], t=t)
        if i == 0 or i == len(self.times):
            raise ValueError('Evaluation boundary outside source coverage.')
        a, b = self.rows[i-1], self.rows[i]
        if b['t'] - a['t'] > MAX_GAP:
            raise ValueError('Interpolation crosses a source gap > 0.20 sim s.')
        fraction = (t - a['t']) / (b['t'] - a['t'])
        return {k: (t if k == 't' else a[k] + fraction*(b[k]-a[k]))
                for k in a if k in b}

    def window(self, start, end):
        if end <= start:
            raise ValueError('Empty evaluation interval.')
        result = [self.at(start)]
        result.extend(r for r in self.rows if start < r['t'] < end)
        result.append(self.at(end))
        if any(b['t']-a['t'] > MAX_GAP for a, b in zip(result, result[1:])):
            raise ValueError('Source gap > 0.20 sim s inside evaluation window.')
        return result


def pose_metrics(rows):
    first, last = rows[0], rows[-1]
    dx, dy = last['x']-first['x'], last['y']-first['y']
    c, s = math.cos(first['yaw_continuous']), math.sin(first['yaw_continuous'])
    signed_path = path = 0.
    for a, b in zip(rows, rows[1:]):
        mx, my = b['x']-a['x'], b['y']-a['y']
        heading = .5*(a['yaw_continuous']+b['yaw_continuous'])
        signed_path += math.cos(heading)*mx + math.sin(heading)*my
        path += math.hypot(mx, my)
    return dict(forward_m=c*dx+s*dy, lateral_m=-s*dx+c*dy,
                endpoint_distance_m=math.hypot(dx, dy), path_m=path,
                signed_path_m=signed_path,
                yaw_change_rad=last['yaw_continuous']-first['yaw_continuous'])


def velocities(rows, start, end):
    values = []
    for a, b in zip(rows, rows[1:]):
        dt = b['t']-a['t']
        mid = .5*(a['t']+b['t'])
        if start <= mid <= end:
            heading = .5*(a['yaw_continuous']+b['yaw_continuous'])
            dx, dy = b['x']-a['x'], b['y']-a['y']
            values.append(dict(t=mid, dt=dt,
                               v=(math.cos(heading)*dx+math.sin(heading)*dy)/dt,
                               speed=math.hypot(dx, dy)/dt,
                               w=(b['yaw_continuous']-a['yaw_continuous'])/dt))
    if not values:
        raise ValueError('No velocity samples in plateau.')
    return values


def mean_velocity(values, key):
    return sum(r[key]*r['dt'] for r in values)/sum(r['dt'] for r in values)


def stopping(rows, stop_t, end):
    # Require a sustained 0.5 sim s below both thresholds after zero command.
    # Pose-derived speed includes lateral motion; /odom twist is not GT.
    values = velocities(rows, stop_t, end)
    candidate = None
    stop_at = None
    for r in values:
        if r['speed'] < .01 and abs(r['w']) < .02:
            candidate = r['t'] if candidate is None else candidate
            if r['t'] - candidate >= .5:
                stop_at = candidate
                break
        else:
            candidate = None
    braking = Series(rows).window(stop_t, stop_at if stop_at else end)
    return dict(stopped=stop_at is not None,
                stop_time_s=None if stop_at is None else max(0., stop_at-stop_t),
                stop_distance_m=pose_metrics(braking)['path_m'],
                translation_threshold_mps=.01, angular_threshold_rad_s=.02,
                sustained_s=.5)


def wheel_metrics(series, start, end, radius, separation):
    # Unwrap before interpolating boundary angles; never interpolate raw wraps.
    unwrapped = []
    totals = [0., 0.]
    event_times = [[], []]
    for i, row in enumerate(series.rows):
        out = {'t': row['t']}
        for j, side in enumerate(('left', 'right')):
            if i:
                raw = row[side+'_q'] - series.rows[i-1][side+'_q']
                inc = wrap_angle(raw)
                if abs(raw-inc) > math.pi:
                    event_times[j].append(row['t'])
                totals[j] += inc
            out[side+'_q'] = totals[j]
        unwrapped.append(out)
    points = Series(unwrapped).window(start, end)
    left = points[-1]['left_q'] - points[0]['left_q']
    right = points[-1]['right_q'] - points[0]['right_q']
    return dict(left_angle_rad=left, right_angle_rad=right,
                signed_path_m=.5*radius*(left+right),
                yaw_change_rad=radius*(right-left)/separation,
                wrap_events_left=sum(start < t <= end for t in event_times[0]),
                wrap_events_right=sum(start < t <= end for t in event_times[1]))


def evaluate(gt, odom, joints, motion, settle, commands, radius, separation):
    start, stop, end = motion['start'], motion['end'], settle['end']
    Series(commands).window(start, end)
    g_rows, o_rows = gt.window(start, end), odom.window(start, end)
    g, o = pose_metrics(g_rows), pose_metrics(o_rows)
    # Use post-ramp plateau, not displacement/duration, to assess tracking.
    # Nominal worst-case ramp time + 0.4 s; at least 0.3 s plateau required.
    ramp = max(abs(motion['v'])/.60, abs(motion['w'])/1.80) + .40
    plateau_start, plateau_end = start+ramp, stop-.15
    if plateau_end-plateau_start < .3:
        raise ValueError('Command duration too short for plateau.')
    plateau = velocities(g_rows, plateau_start, plateau_end)
    v, w = mean_velocity(plateau, 'v'), mean_velocity(plateau, 'w')
    # Record ideal request integral from actual published samples; includes
    # phase overshoot/publication jitter, but deliberately no ramp model.
    integral = {'linear_m': 0., 'yaw_rad': 0.}
    for a, b in zip(commands, commands[1:]):
        dt = max(0., min(end, b['t']) - max(start, a['t']))
        integral['linear_m'] += a['requested_linear_mps']*dt
        integral['yaw_rad'] += a['requested_angular_rad_s']*dt
    wheels = wheel_metrics(joints, start, end, radius, separation)
    error = dict(translation_vector_error_m=math.hypot(
        o['forward_m']-g['forward_m'], o['lateral_m']-g['lateral_m']),
        signed_path_error_m=o['signed_path_m']-g['signed_path_m'],
        heading_error_rad=o['yaw_change_rad']-g['yaw_change_rad'],
        endpoint_heading_error_rad=wrap_angle(o['yaw_change_rad']-g['yaw_change_rad']))
    candidates = {}
    if motion['kind'] == 'linear' and abs(g['signed_path_m']) > .05:
        if abs(o['signed_path_m']) > .05 and o['signed_path_m']*g['signed_path_m'] > 0:
            candidates['wheel_radius_m'] = radius*g['signed_path_m']/o['signed_path_m']
    if motion['kind'] == 'angular' and abs(g['yaw_change_rad']) > .2:
        if o['yaw_change_rad']*g['yaw_change_rad'] > 0:
            candidates['wheel_separation_m'] = separation*o['yaw_change_rad']/g['yaw_change_rad']
    return dict(name=motion['name'], kind=motion['kind'], repeat=motion['repeat'],
                window=dict(start=start, zero_command=stop, end=end),
                command_tracking=dict(requested_v_mps=motion['v'], requested_w_rad_s=motion['w'],
                    measured_v_mps=v, measured_w_rad_s=w,
                    v_error_mps=v-motion['v'], w_error_rad_s=w-motion['w'],
                    v_ratio=None if not motion['v'] else v/motion['v'],
                    w_ratio=None if not motion['w'] else w/motion['w'],
                    plateau_start=plateau_start, plateau_end=plateau_end,
                    request_integral=integral,
                    note='Request integral ignores acceleration; it is diagnostic, not acceptance.'),
                isaac_ground_truth=g, ros_odometry=o, odom_minus_ground_truth=error,
                wheel_position_evidence=wheels, stopping=stopping(g_rows, stop, end),
                calibration_candidates=candidates)


def analyze(output):
    output = Path(output)
    report = json.loads((output/'manifest.json').read_text())
    evaluations, problems = [], []
    try:
        gt = Series(read_csv(output/'isaac.csv'))
        odom = Series(read_csv(output/'ros_odometry.csv'))
        joints = Series(read_csv(output/'ros_joint_states.csv'))
        command_rows = read_csv(output/'commands.csv')
        # Multiple wall-clock publications can share a simulation stamp.
        # The last publication at that stamp is the held request thereafter.
        commands = []
        for row in command_rows:
            if commands and row['t'] == commands[-1]['t']:
                commands[-1] = row
            else:
                commands.append(row)
        commands = Series(commands).rows
        phases = report['phases']
        geometry = report.get('live_parameters', {}).get('odometry', {})
        radius = geometry.get('wheel_radius', RADIUS)
        separation = geometry.get('wheel_separation', SEPARATION)
        for i, phase in enumerate(phases):
            if phase['kind'] == 'idle':
                continue
            try:
                if i+1 >= len(phases) or phases[i+1]['trial'] != phase['trial']:
                    raise ValueError('Matching settle phase was not completed.')
                result = evaluate(gt, odom, joints, phase, phases[i+1], commands, radius, separation)
                evaluations.append(result)
                if not result['stopping']['stopped']:
                    problems.append(phase['name'] + ': robot did not settle below thresholds.')
            except ValueError as exc:
                problems.append(phase['name'] + ': ' + str(exc))
        for repeat in range(1, report['repeats']+1):
            square = [p for p in phases if p['repeat'] == repeat and p['kind'].startswith('square_')]
            if square:
                if len(square) != 8:
                    problems.append(f'Square repeat {repeat}: incomplete trajectory.')
                    continue
                start = square[0]['start']
                index = phases.index(square[-1])
                if index+1 >= len(phases):
                    problems.append(f'Square repeat {repeat}: missing final settle.')
                    continue
                end = phases[index+1]['end']
                try:
                    g, o = pose_metrics(gt.window(start, end)), pose_metrics(odom.window(start, end))
                    report.setdefault('square_closure', []).append(dict(repeat=repeat,
                        isaac_ground_truth=g, ros_odometry=o,
                        endpoint_translation_error_m=math.hypot(o['forward_m']-g['forward_m'],
                                                                 o['lateral_m']-g['lateral_m']),
                        accumulated_heading_error_rad=o['yaw_change_rad']-g['yaw_change_rad'],
                        ground_truth_closure_heading_rad=wrap_angle(g['yaw_change_rad'])))
                except ValueError as exc:
                    problems.append(f'Square repeat {repeat}: {exc}')
    except (ValueError, OSError, KeyError) as exc:
        problems.append(str(exc))
    if len(evaluations) != report.get('planned_maneuvers', 0):
        problems.append('Completed/evaluable maneuver count differs from planned count.')
    if report.get('error'):
        problems.append(report['error'])
    report['status'] = 'incomplete' if problems else 'recorded_pending_review'
    report['analysis_problems'] = problems
    report['maneuvers'] = evaluations
    grouped = {}
    for item in evaluations:
        track = item['command_tracking']
        key = item["name"].rsplit("_r", 1)[0]
        grouped.setdefault(key, []).append(item)
    report['repeatability'] = {}
    for key, items in grouped.items():
        metrics = {}
        for field, getter in (
            ('actual_v_mps', lambda i: i['command_tracking']['measured_v_mps']),
            ('actual_w_rad_s', lambda i: i['command_tracking']['measured_w_rad_s']),
            ('translation_error_m', lambda i: i['odom_minus_ground_truth']['translation_vector_error_m']),
            ('heading_error_rad', lambda i: i['odom_minus_ground_truth']['heading_error_rad'])):
            values = [getter(i) for i in items]
            metrics[field] = dict(mean=statistics.mean(values),
                                 stdev=statistics.stdev(values) if len(values)>1 else None,
                                 min=min(values), max=max(values))
        report['repeatability'][key] = dict(repeats=len(items), metrics=metrics)
    report['calibration_advisory'] = {
        'note': 'Candidates only. Review command tracking, direction/speed dependence and slip first. '
                'Radius affects heading too; re-evaluate separation after any radius change. No parameters modified.'}
    for parameter in ('wheel_radius_m', 'wheel_separation_m'):
        values = [i['calibration_candidates'][parameter] for i in evaluations
                  if parameter in i['calibration_candidates']]
        if values:
            report['calibration_advisory'][parameter] = dict(
                median=statistics.median(values), min=min(values), max=max(values), runs=len(values))
    save_json(output/'report.json', report)
    flat = []
    for i in evaluations:
        t, e, stop = i['command_tracking'], i['odom_minus_ground_truth'], i['stopping']
        flat.append(dict(name=i['name'], requested_v=t['requested_v_mps'], actual_v=t['measured_v_mps'],
                         requested_w=t['requested_w_rad_s'], actual_w=t['measured_w_rad_s'],
                         translation_error_m=e['translation_vector_error_m'],
                         heading_error_deg=math.degrees(e['heading_error_rad']),
                         lateral_drift_m=i['isaac_ground_truth']['lateral_m'],
                         stop_time_s=stop['stop_time_s'], stop_distance_m=stop['stop_distance_m'],
                         left_wraps=i['wheel_position_evidence']['wrap_events_left'],
                         right_wraps=i['wheel_position_evidence']['wrap_events_right']))
    save_csv(output/'summary.csv', flat)
    text = ['# TurtleBot 4 Step 8B odometry test', '', f"Status: **{report['status']}**", '',
            'Recorded results require review; this script does not mark Step 8 accepted.', '',
            '| Maneuver | v req / actual (m/s) | w req / actual (rad/s) | Odom position error (mm) | Odom heading error (deg) |',
            '|---|---:|---:|---:|---:|']
    for r in flat:
        text.append(f"| {r['name']} | {r['requested_v']:.3f} / {r['actual_v']:.3f} | "
                    f"{r['requested_w']:.3f} / {r['actual_w']:.3f} | "
                    f"{1000*r['translation_error_m']:.3f} | {r['heading_error_deg']:.4f} |")
    text += ['', '## Data limitations', ''] + (['- '+p for p in problems] or ['No detected coverage or completion errors.'])
    (output/'report.md').write_text('\n'.join(text)+'\n')
    print(f"Status: {report['status']}; evaluated {len(evaluations)} maneuvers. Results: {output}")
    return report


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
    robot = view.create_articulation_view('/World/turtlebot4_standard/Geometry/base_link')
    assert robot.count == 1, f'Expected one robot, found {robot.count}'

    links = list(robot.shared_metatype.link_names)
    assert links and 'base_link' in links, (
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

    previous_task = globals().get('asn_turtlebot4_recording_task')
    if previous_task is not None and not previous_task.done():
        raise RuntimeError('Step 8B recorder already armed; wait or cancel it first.')

    folder = Path(tempfile.mkdtemp(prefix='asn_step8b_'))
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

                if not omni.timeline.get_timeline_interface().is_playing():
                    raise RuntimeError('Isaac timeline paused/stopped; recording invalid.')

                if not active and (folder / 'run').exists():
                    active = True
                    request = json.loads((folder / 'run').read_text())
                    active_deadline = now + float(request['wall_budget_s'])
                    save_json(folder / 'active.json', {'status': 'recording'})
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

    globals()['asn_turtlebot4_recording_task'] = asyncio.ensure_future(record())

    print('STEP 8B ISAAC RECORDER ARMED (300 seconds).')
    print('Root ground truth: /World/turtlebot4_standard/Geometry/base_link')
    print('Passive recorder only; no pose, target, gain, physics, or TF changes.')


def frozen_source(project):
    import subprocess
    paths = ('ros2_ws/src/asn_simulation/scripts/asn_wheel_odom_unwrapped.py',)
    result = {'files': {}}
    for path in paths:
        file = project/path
        if file.exists():
            raw = file.read_bytes()
            result['files'][path] = dict(sha256=hashlib.sha256(raw).hexdigest(),
                                        content=raw.decode())
    for name, command in (('git_head', ['git', 'rev-parse', 'HEAD']),
                          ('git_status', ['git', 'status', '--short'])):
        try:
            result[name] = subprocess.check_output(command, cwd=project, text=True,
                                                  stderr=subprocess.DEVNULL).strip()
        except (OSError, subprocess.CalledProcessError):
            result[name] = None
    return result


def run_ros(args):
    import shutil
    import signal
    from datetime import datetime, timezone
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from controller_manager_msgs.srv import ListControllers
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from rcl_interfaces.srv import GetParameters

    planned = sequence(args.scenario, args.repeats)
    project = args.project.expanduser().resolve()
    if not project.is_dir():
        raise RuntimeError(f'Project not found: {project}')
    if not SESSION.exists():
        raise RuntimeError('Run this file in Isaac Script Editor first.')
    session = json.loads(SESSION.read_text())
    folder = Path(session['folder'])
    if time.time() > session['expires'] or (folder/'finished.json').exists():
        raise RuntimeError('Recorder expired/finished; arm again in Isaac.')
    output = project/'results/validation/TurtleBot4'/(
        args.scenario+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    output.mkdir(parents=True)
    manifest = dict(scenario=args.scenario, repeats=args.repeats, planned_phases=planned,
                    planned_maneuvers=sum(p['kind']!='idle' for p in planned), phases=[],
                    ground_truth_source='/World/turtlebot4_standard/Geometry/base_link articulation root',
                    odometry_source=ODOM, command_topic=CMD,
                    max_interpolation_gap_sim_s=MAX_GAP,
                    source_snapshot=frozen_source(project), error=None)
    joints, odom, commands = [], [], []
    state = dict(t=None, stamp=None, advanced=time.monotonic(), error=None,
                 joint_arrival=None, odom_arrival=None)
    publisher, started, failure = None, False, None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('asn_turtlebot4_validation_'+str(int(time.time())))

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    old_term = signal.signal(signal.SIGTERM, interrupt)

    def seconds(stamp):
        return stamp.sec+stamp.nanosec*1e-9

    def on_clock(msg):
        t = seconds(msg.clock)
        if state['t'] is not None and t < state['t']:
            state['error'] = 'Simulation clock reset.'
        if t != state['t']:
            state['advanced'] = time.monotonic()
        state.update(t=t, stamp=msg.clock)

    def on_joint(msg):
        if not all(name in msg.name for name in WHEELS):
            state['error'] = 'Required wheel joints missing.'
            return
        row = {'t': seconds(msg.header.stamp)}
        for side, name in zip(('left', 'right'), WHEELS):
            i = msg.name.index(name)
            if i >= len(msg.position):
                state['error'] = 'Wheel position missing.'
                return
            row[side+'_q'] = float(msg.position[i])
            if i < len(msg.velocity):
                row[side+'_v'] = float(msg.velocity[i])
        if not all(math.isfinite(v) for v in row.values()):
            state['error'] = 'Non-finite wheel feedback.'
            return
        if joints and row['t'] < joints[-1]['t']:
            state['error'] = 'Joint timestamps moved backwards.'
        joints.append(row)
        state['joint_arrival'] = time.monotonic()

    def on_odom(msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            state['error'] = 'Expected /odom frames odom -> base_link.'
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        row = dict(t=seconds(msg.header.stamp), x=float(p.x), y=float(p.y),
                   yaw=yaw_from_xyzw(q.x, q.y, q.z, q.w),
                   v=float(msg.twist.twist.linear.x), omega=float(msg.twist.twist.angular.z))
        if not all(math.isfinite(v) for v in row.values()):
            state['error'] = 'Non-finite odometry.'
            return
        if odom and row['t'] < odom[-1]['t']:
            state['error'] = 'Odometry timestamps moved backwards.'
        odom.append(row)
        state['odom_arrival'] = time.monotonic()

    def publish(v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        publisher.publish(msg)
        commands.append(dict(t=state['t'], wall_monotonic_s=time.monotonic(),
                             requested_linear_mps=v, requested_angular_rad_s=w))

    def health():
        if state['error']:
            raise RuntimeError(state['error'])
        now = time.monotonic()
        if now-state['advanced'] > 3.:
            raise RuntimeError('Simulation clock stalled (>3 wall s).')
        for name, rows in (('joint', joints), ('odom', odom)):
            arrival = state[name+'_arrival']
            if arrival is None or now-arrival > 2.:
                raise RuntimeError(name+' feedback stopped arriving (>2 wall s).')
            if abs(rows[-1]['t']-state['t']) > .5:
                raise RuntimeError(name+' timestamp stale or not on simulation clock.')
        heartbeat = folder/'heartbeat'
        if not heartbeat.exists() or time.time()-heartbeat.stat().st_mtime > 3.:
            raise RuntimeError('Isaac recorder heartbeat missing/stale.')
        if (folder/'finished.json').exists():
            raise RuntimeError('Isaac recorder ended early.')

    def graph_check():
        for topic in ('/clock', '/joint_states', ODOM):
            if node.count_publishers(topic) != 1:
                raise RuntimeError(f'Expected one publisher on {topic}.')
        others = [p.node_name for p in node.get_publishers_info_by_topic(CMD)
                  if (p.node_name, p.node_namespace) != (node.get_name(), node.get_namespace())]
        if others:
            raise RuntimeError(f'Competing command publisher(s): {others}. Stop Nav2/teleop/mission output.')

    def parameters(node_name, names):
        client = node.create_client(GetParameters, node_name+'/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('Parameter service unavailable: '+node_name)
            request = GetParameters.Request()
            request.names = names
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            if not future.done() or future.result() is None:
                raise RuntimeError('Parameter query timed out: '+node_name)
            result = {}
            fields = {1:'bool_value', 2:'integer_value', 3:'double_value', 4:'string_value'}
            for name, value in zip(names, future.result().values):
                if value.type not in fields:
                    raise RuntimeError('Unset/unsupported live parameter: '+node_name+'/'+name)
                result[name] = getattr(value, fields[value.type])
            return result
        finally:
            node.destroy_client(client)

    try:
        node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
        node.create_subscription(JointState, '/joint_states', on_joint, qos_profile_sensor_data)
        node.create_subscription(Odometry, ODOM, on_odom, qos_profile_sensor_data)
        # TurtleBot4 Isaac OmniGraph publishes plain Twist, not ros2_control.
        # Keep wheel-derived odometry with exactly one odom/TF owner.
        live = dict(odometry=parameters('/asn_wheel_odom_unwrapped',
            ['wheel_radius', 'wheel_separation', 'odom_topic',
             'odom_frame', 'base_frame', 'publish_tf']))
        manifest['live_parameters'] = live
        wheel = live['odometry']
        if (wheel['odom_topic'], wheel['odom_frame'], wheel['base_frame']) != (ODOM, 'odom', 'base_link'):
            raise RuntimeError('Unexpected wheel-odometry topic/frames.')
        if not wheel['publish_tf']:
            raise RuntimeError('Wheel odometry must own odom -> base_link TF.')
        if not math.isclose(wheel['wheel_radius'], RADIUS, abs_tol=1e-6) or not math.isclose(wheel['wheel_separation'], SEPARATION, abs_tol=1e-6):
            raise RuntimeError('Live wheel geometry differs from nominal TurtleBot4 0.03575/0.233 m.')
        print('Live baseline parameters: '+json.dumps(live), flush=True)
        deadline = time.monotonic()+8.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if state['t'] is not None and len(joints)>=2 and len(odom)>=2:
                break
        if state['t'] is None or len(joints)<2 or len(odom)<2:
            raise RuntimeError('Clock, joint positions, or /odom unavailable.')
        graph_check()
        if node.count_publishers(CMD):
            raise RuntimeError('A command publisher exists; isolate this test first.')
        health()
        publisher = node.create_publisher(Twist, CMD, 10)
        deadline = time.monotonic()+5.
        while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if publisher.get_subscription_count()==0:
            raise RuntimeError('No matching Twist subscriber on /cmd_vel.')
        save_json(folder/'run', dict(wall_budget_s=sum(p['duration'] for p in planned)*8.+120.))
        started = True
        deadline = time.monotonic()+5.
        while not (folder/'active.json').exists() and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            health()
        if not (folder/'active.json').exists():
            raise RuntimeError('Isaac recorder did not acknowledge start.')
        next_graph = 0.
        for phase in planned:
            phase = dict(phase, start=state['t'])
            end = phase['start']+phase['duration']
            next_send = 0.
            wall_deadline = time.monotonic()+max(30., phase['duration']*8.)
            manifest['active_phase'] = phase
            save_json(output/'manifest.json', manifest)
            print(f"{phase['name']}: {phase['duration']:.2f} sim s, v={phase['v']:+.2f}, w={phase['w']:+.2f}", flush=True)
            while state['t'] < end:
                rclpy.spin_once(node, timeout_sec=.005)
                health()
                now = time.monotonic()
                if now > wall_deadline:
                    raise RuntimeError('Phase wall deadline exceeded.')
                if now >= next_graph:
                    graph_check()
                    next_graph = now+.5
                if state['t'] >= end:
                    break
                if now >= next_send:
                    publish(phase['v'], phase['w'])
                    next_send = now+.02  # wall cadence preserves controller expiry even with low RTF
            publish(0., 0.)
            phase['end'] = state['t']
            manifest['phases'].append(phase)
            manifest.pop('active_phase', None)
    except BaseException as exc:
        failure = 'Interrupted' if isinstance(exc, KeyboardInterrupt) else str(exc)
        print('ABORTED: '+failure, flush=True)
    finally:
        if publisher is not None and state['stamp'] is not None:
            try:
                deadline = time.monotonic()+.5
                while time.monotonic()<deadline:
                    publish(0., 0.)
                    rclpy.spin_once(node, timeout_sec=.02)
            except BaseException:
                pass
        # Release the armed session even if preflight failed before movement.
        (folder/'stop').write_text('stop')
        save_csv(output/'ros_joint_states.csv', joints)
        save_csv(output/'ros_odometry.csv', odom)
        save_csv(output/'commands.csv', commands)
        manifest['error'] = failure
        save_json(output/'manifest.json', manifest)
        signal.signal(signal.SIGTERM, old_term)
        node.destroy_node()
        rclpy.shutdown()
    if started:
        deadline = time.monotonic()+5.
        while not (folder/'finished.json').exists() and time.monotonic()<deadline:
            time.sleep(.05)
        for name in ('isaac.csv', 'finished.json'):
            if (folder/name).exists():
                shutil.copy2(folder/name, output/name)
        if (output/'finished.json').exists():
            finished = json.loads((output/'finished.json').read_text())
            if finished['status'] != 'ok':
                manifest['error'] = manifest['error'] or finished['error']
        else:
            manifest['error'] = manifest['error'] or 'Isaac recorder did not finalize.'
        save_json(output/'manifest.json', manifest)
    report = analyze(output)
    return 1 if report['status']=='incomplete' else 0


def self_test():
    # These checks target analysis errors that could produce false calibration.
    import tempfile
    def rows(v=.4, w=0., origin=(0.,0.,0.), offset=0.):
        out = []
        x0,y0,yaw0 = origin
        for i in range(1001):
            t = offset+i*.01
            d = max(0., min(t-1., 3.))
            yaw = yaw0+w*d
            if w:
                x = x0+v/w*(math.sin(yaw)-math.sin(yaw0))
                y = y0-v/w*(math.cos(yaw)-math.cos(yaw0))
            else:
                x,y = x0+v*d*math.cos(yaw0), y0+v*d*math.sin(yaw0)
            out.append(dict(t=t,x=x,y=y,yaw=wrap_angle(yaw)))
        return out
    a,b = Series(rows()), Series(rows(origin=(3.,-2.,1.2), offset=.003))
    ga,ob = pose_metrics(a.window(1.1,6.)),pose_metrics(b.window(1.1,6.))
    assert abs(ga['forward_m']-ob['forward_m']) < 1e-9
    assert abs(ob['lateral_m']) < 1e-9
    spin = Series(rows(v=0.,w=2.5))
    assert abs(pose_metrics(spin.window(1.,6.))['yaw_change_rad']-7.5) < 1e-9
    curved = pose_metrics(Series(rows(v=.25,w=.75)).window(1.,6.))
    assert abs(curved['signed_path_m']-.75)<1e-5
    reversed_motion = pose_metrics(Series(rows(v=-.4)).window(1.,6.))
    assert abs(reversed_motion['signed_path_m']+1.2)<1e-9
    assert stopping(a.window(1.,6.),4.,6.)['stopped']
    try:
        Series([dict(t=0.,x=0.),dict(t=1.,x=1.)]).window(.1,.9)
    except ValueError:
        pass
    else:
        raise AssertionError('Missing-data gap accepted.')
    wrapped = []
    for i in range(701):
        t=i*.01
        q=10.*t
        # +/-4pi discontinuity from continuous-joint representation.
        q=(q+2*math.pi)%(4*math.pi)-2*math.pi
        wrapped.append(dict(t=t,left_q=q,right_q=q))
    wh=wheel_metrics(Series(wrapped),0.,7.,RADIUS,SEPARATION)
    assert abs(wh['signed_path_m']-RADIUS*70.)<1e-8
    assert wh['wrap_events_left']>0
    motion=dict(name='forward_r1',kind='linear',repeat=1,start=1.1,end=4.,v=.4,w=0.)
    settle=dict(end=6.)
    joints=[dict(t=r['t'],left_q=.4*max(0.,min(r['t']-1.,3.))/RADIUS,
                 right_q=.4*max(0.,min(r['t']-1.,3.))/RADIUS) for r in rows()]
    commands=[dict(t=i*.01,requested_linear_mps=.4 if 1.<=i*.01<4. else 0.,
                   requested_angular_rad_s=0.) for i in range(701)]
    result=evaluate(a,b,Series(joints),motion,settle,commands,RADIUS,SEPARATION)
    assert result['odom_minus_ground_truth']['translation_vector_error_m']<1e-8
    assert abs(result['command_tracking']['v_ratio']-1.)<1e-8
    assert abs(result['calibration_candidates']['wheel_radius_m']-RADIUS)<1e-8
    scaled=Series([dict(r,x=r['x']*1.10,y=r['y']*1.10) for r in rows()])
    result=evaluate(a,scaled,Series(joints),motion,settle,commands,RADIUS,SEPARATION)
    assert abs(result['calibration_candidates']['wheel_radius_m']-RADIUS/1.10)<1e-8
    turn=Series(rows(v=0.,w=1.1))
    scaled_turn=Series([dict(r,yaw=wrap_angle(1.1*1.05*max(0.,min(r['t']-1.,3.)))) for r in rows(v=0.,w=1.1)])
    angular=dict(motion,kind='angular',v=0.,w=1.1)
    turn_commands=[dict(r,requested_linear_mps=0.,requested_angular_rad_s=1.1 if 1.<=r['t']<4. else 0.) for r in commands]
    result=evaluate(turn,scaled_turn,Series(joints),angular,settle,turn_commands,RADIUS,SEPARATION)
    assert abs(result['calibration_candidates']['wheel_separation_m']-SEPARATION*1.05)<1e-8
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary)
        save_csv(root/'isaac.csv',rows())
        save_csv(root/'ros_odometry.csv',rows(origin=(3.,-2.,1.2),offset=.003))
        save_csv(root/'ros_joint_states.csv',joints)
        save_csv(root/'commands.csv',commands)
        save_json(root/'manifest.json',dict(phases=[motion,dict(settle,kind='idle',trial=None,repeat=1)],
                                          repeats=1,planned_maneuvers=1,error=None))
        # settle's trial must match the maneuver to be accepted.
        data=json.loads((root/'manifest.json').read_text())
        data['phases'][0]['trial']='forward_r1'
        data['phases'][1]['trial']='forward_r1'
        save_json(root/'manifest.json',data)
        report=analyze(root)
        assert report['status']=='recorded_pending_review', report['analysis_problems']
        data['error']='Injected abort'
        save_json(root/'manifest.json',data)
        assert analyze(root)['status']=='incomplete'
        assert (root/'summary.csv').exists() and (root/'report.md').exists()
    print('Offline analysis checks passed (synthetic data only).')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--ros',action='store_true')
    mode.add_argument('--plan',action='store_true')
    mode.add_argument('--self-test',action='store_true')
    mode.add_argument('--analyze',type=Path,metavar='RESULTS_DIR')
    parser.add_argument('--scenario',choices=('smoke','linear','angular','sweep','arc','square','all'),default='smoke')
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--project',type=Path,default=Path.home()/'projects/autonomous-semantic-navigation')
    args=parser.parse_args()
    if not 1 <= args.repeats <= 10:
        parser.error('--repeats must be between 1 and 10.')
    if args.self_test:
        self_test()
    elif args.analyze:
        return int(analyze(args.analyze.expanduser())['status']=='incomplete')
    elif args.plan:
        phases=sequence(args.scenario,args.repeats)
        print(json.dumps(dict(scenario=args.scenario,repeats=args.repeats,
                              duration_sim_s=sum(p['duration'] for p in phases),phases=phases),indent=2))
    elif args.ros:
        return run_ros(args)
    else:
        parser.error('In a terminal, specify --ros, --plan, --self-test or --analyze. Arm with no arguments inside Isaac.')
    return 0


# Isaac supplies Kit arguments and may execute with a non-main module name.
# Offline imports never arm a recorder; explicit terminal modes use argparse.
_MODES = ('--ros', '--plan', '--self-test', '--analyze')
if 'omni.kit.app' in sys.modules and not any(mode in sys.argv for mode in _MODES):
    start_isaac()
elif __name__ == '__main__':
    raise SystemExit(main())
