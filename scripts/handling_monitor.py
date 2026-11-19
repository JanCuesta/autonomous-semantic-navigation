#!/usr/bin/env python3
"""Read-only TurtleBot 4 handling monitor (laptop/Pi) and atomic test markers.

JSONL stdout: redirect to a different log per process. Requires ROS 2 only for
ROS observations; system/network checks continue if ROS is unavailable.
"""

import argparse
import datetime as dt
import glob
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

STOP = threading.Event()
WRITE_LOCK = threading.Lock()
JOURNAL = None
TOPICS = {
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/oakd/rgb/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
    '/oakd/stereo/image_raw/compressedDepth': 'sensor_msgs/msg/CompressedImage',
}
STATUS_TOPICS = {
    '/battery_state', '/dock_status', '/hazard_detection',
    '/wheel_status', '/kidnap_status',
}
JOURNAL_PATTERN = re.compile(
    r'usb|xhci|under.?voltage|low.?voltage|throttl|networkmanager|hostapd|'
    r'wpa_supplicant|turtlebot|rplidar|x_link|depthai|brownout|power',
    re.IGNORECASE,
)


def utc_time():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec='milliseconds')


def boot_id():
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return 'unavailable'


def emit(kind, **values):
    record = {
        'time': utc_time(), 'epoch': round(time.time(), 3),
        'monotonic': round(time.monotonic(), 3), 'kind': kind, **values,
    }
    with WRITE_LOCK:
        # One write per record. Redirect each logger to its *own* file.
        os.write(1, (json.dumps(record, default=str, separators=(',', ':')) + '\n').encode())


def run_command(argv, timeout=3):
    try:
        p = subprocess.run(
            argv, capture_output=True, text=True, timeout=timeout,
            env={**os.environ, 'LC_ALL': 'C'},
        )
        return {'rc': p.returncode, 'output': (p.stdout + p.stderr).strip()[:5000]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'rc': None, 'output': str(exc)}


def lidar_devices():
    paths = ['/dev/RPLIDAR']
    for expression in ('/dev/rplidar*', '/dev/ttyUSB*', '/dev/serial/by-id/*'):
        paths.extend(glob.glob(expression))
    result = {}
    for value in sorted(set(paths)):
        path = Path(value)
        if path.exists() or path.is_symlink():
            result[value] = {'exists': path.exists(), 'target': os.path.realpath(value)}
    return {'rplidar_present': Path('/dev/RPLIDAR').exists(), 'devices': result}


def ros_monitor(role, no_camera):
    node = None
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        rclpy.init()
        node = Node('handling_monitor_' + role + '_' + str(os.getpid()))
        rate_topics = {key: typ for key, typ in TOPICS.items()
                       if not (no_camera and key != '/scan')}
        counts = {topic: 0 for topic in rate_topics}
        subscriptions = {}
        previous_status = {}
        last_status_emit_at = {}

        def receive_rate(topic):
            def callback(_msg):
                counts[topic] += 1
            return callback

        def receive_status(topic):
            def callback(msg):
                try:
                    obj = message_to_ordereddict(msg)
                    if isinstance(obj, dict):
                        obj.pop('header', None)
                    if previous_status.get(topic) != obj:
                        # Battery/encoder data may jitter at high frequency.
                        # Never throttle discrete dock/lift/hazard transitions.
                        quiet_seconds = {'/battery_state': 10.0,
                                         '/wheel_status': 5.0}.get(topic, 0.0)
                        now = time.monotonic()
                        if now - last_status_emit_at.get(topic, -1e10) >= quiet_seconds:
                            previous_status[topic] = obj
                            last_status_emit_at[topic] = now
                            emit('status', topic=topic, value=obj)
                except Exception as exc:
                    emit('ros_status_error', topic=topic, error=str(exc))
            return callback

        # Count arrivals on BOTH hosts. Only Pi-side rates can distinguish
        # sensor/driver failure from a laptop receive/transport failure.
        for topic, typ in rate_topics.items():
            try:
                subscriptions[topic] = node.create_subscription(
                    get_message(typ), topic, receive_rate(topic), qos_profile_sensor_data)
            except Exception as exc:
                emit('ros_topic_error', topic=topic, error=str(exc))
        emit('ros_started', role=role, rate_topics=list(rate_topics),
             note='Extra subscribers add load, especially compressed images.')
        next_discovery = 0.0
        window_start = time.monotonic()
        while not STOP.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if role == 'pi' and now >= next_discovery:
                try:
                    discovered = {name: types for name, types in node.get_topic_names_and_types()}
                    for topic in STATUS_TOPICS:
                        if topic not in discovered or topic in subscriptions:
                            continue
                        typ = discovered[topic][0]
                        try:
                            subscriptions[topic] = node.create_subscription(
                                get_message(typ), topic, receive_status(topic),
                                qos_profile_sensor_data)
                            emit('status_subscribed', topic=topic, type=typ)
                        except Exception as exc:
                            emit('status_unavailable', topic=topic, type=typ, error=str(exc))
                except Exception as exc:
                    emit('ros_discovery_error', error=str(exc))
                next_discovery = now + 15.0
            if now - window_start >= 10.0:
                elapsed = now - window_start
                emit('rates', role=role, seconds=round(elapsed, 2),
                     counts=dict(counts),
                     hz={topic: round(count / elapsed, 2) for topic, count in counts.items()},
                     note='0 Hz means subscriber received no messages; not proof the driver stopped.')
                counts = {key: 0 for key in counts}
                window_start = now
    except Exception as exc:
        emit('ros_unavailable' if node is None else 'ros_error', error=str(exc))
    finally:
        if node is not None:
            try:
                node.destroy_node()
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as exc:
                emit('ros_cleanup_error', error=str(exc))


def journal_monitor():
    global JOURNAL
    # journalctl follows without restarting/changing any services.
    coverage = {
        'system': run_command(['journalctl', '-b', '-n', '1', '--no-pager'], timeout=3),
        'kernel': run_command(['journalctl', '-b', '-k', '-n', '1', '--no-pager'], timeout=3),
    }
    emit('journal_coverage', **{
        name: {'accessible': data['rc'] == 0,
               'detail': data['output'][:220]} for name, data in coverage.items()
    })
    try:
        proc = subprocess.Popen(
            ['journalctl', '-f', '-n', '0', '-o', 'short-iso-precise', '--no-pager'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            bufsize=1,
        )
        JOURNAL = proc
        emit('journal_started', pid=proc.pid)
        for line in proc.stdout:
            if JOURNAL_PATTERN.search(line) or 'permission denied' in line.lower():
                emit('journal', line=line.rstrip()[:1600])
            if STOP.is_set():
                break
        if not STOP.is_set():
            emit('journal_ended', returncode=proc.poll(),
                 note='Journal ended unexpectedly; its later events may be missing.')
    except OSError as exc:
        emit('journal_unavailable', error=str(exc))
    finally:
        if JOURNAL is not None and JOURNAL.poll() is None:
            JOURNAL.terminate()


def record_marker(path, trial, note):
    if not path:
        raise SystemExit('mark requires --file /path/to/markers.jsonl')
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    item = {
        'time': utc_time(), 'epoch': round(time.time(), 3),
        'monotonic': round(time.monotonic(), 3), 'kind': 'marker',
        'test': trial, 'note': note, 'boot_id': boot_id(),
    }
    # IMPORTANT: markers must NOT be written into a logger's redirected
    # stdout file, because that file is not opened O_APPEND.
    fd = os.open(str(p), os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
    try:
        os.write(fd, (json.dumps(item) + '\n').encode())
    finally:
        os.close(fd)
    print('Marker saved:', trial, note)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=['laptop', 'pi', 'mark'])
    parser.add_argument('test', nargs='?', help='A, B, C, D, E, F for mark role')
    parser.add_argument('note', nargs='?', help='Observation for mark role')
    parser.add_argument('--file', help='Separate markers.jsonl file for mark role')
    parser.add_argument('--interface', default='wlx7820515043d2')
    parser.add_argument('--target', default='192.168.77.1')
    parser.add_argument('--no-camera', action='store_true',
                        help='Count LiDAR only: reduce image traffic added by laptop logger')
    parser.add_argument('--no-ros', action='store_true', help='Skip ROS subscriptions entirely')
    args = parser.parse_args()
    if args.role == 'mark':
        if not args.test or not args.note:
            parser.error('mark requires test and note, e.g. mark A "START docked"')
        record_marker(args.file, args.test, args.note)
        return
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: STOP.set())
    emit('start', role=args.role, boot_id=boot_id(), pid=os.getpid(),
         interface=args.interface if args.role == 'laptop' else None,
         target=args.target if args.role == 'laptop' else None,
         ros_enabled=not args.no_ros,
         camera_counting=not args.no_ros and not args.no_camera)
    if not args.no_ros:
        threading.Thread(target=ros_monitor, args=(args.role, args.no_camera), daemon=True).start()
    if args.role == 'pi':
        threading.Thread(target=journal_monitor, daemon=True).start()
    previous = {}
    clock_offset = time.time() - time.monotonic()
    while not STOP.is_set():
        now_offset = time.time() - time.monotonic()
        if abs(now_offset - clock_offset) > 2.0:
            emit('clock_jump', seconds=round(now_offset - clock_offset, 3))
        clock_offset = now_offset
        if args.role == 'laptop':
            emit('network',
                 ping=run_command(['ping', '-n', '-I', args.interface, '-c', '3',
                                   '-W', '1', args.target], timeout=6),
                 link=run_command(['iw', 'dev', args.interface, 'link'], timeout=3),
                 address=run_command(['ip', '-4', '-brief', 'addr', 'show', 'dev', args.interface]),
                 adapter=run_command(['ip', '-brief', 'link', 'show', 'dev', args.interface]))
            STOP.wait(5.0)
        else:
            try:
                uptime = float(Path('/proc/uptime').read_text().split()[0])
            except (OSError, ValueError, IndexError):
                uptime = None
            emit('heartbeat', boot_id=boot_id(), uptime_seconds=uptime)
            checks = {
                'wifi': ['nmcli', '-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device', 'status'],
                'addresses': ['ip', '-brief', 'address'],
                'clock_sync': ['timedatectl', 'show', '-p', 'NTPSynchronized', '-p', 'NTP'],
                'ap_wireless': ['iw', 'dev'],
                'active_connections': ['nmcli', '-t', '-f', 'NAME,TYPE,DEVICE', 'connection', 'show', '--active'],
                'usb_tree': ['lsusb', '-t'],
                'usb_list': ['lsusb'],
                'power_throttle': ['vcgencmd', 'get_throttled'],
                'hostapd_state': ['systemctl', 'is-active', 'hostapd'],
                'turtlebot_service': ['systemctl', 'is-active', 'turtlebot4.service'],
                'failed_services': ['systemctl', '--failed', '--no-legend', '--plain'],
            }
            for name, argv in checks.items():
                result = run_command(argv)
                if previous.get(name) != result:
                    emit('pi_sample', check=name, **result)
                    previous[name] = result
                if STOP.is_set():
                    break
            devices = lidar_devices()
            if previous.get('lidar_devices') != devices:
                emit('pi_sample', check='lidar_devices', **devices)
                previous['lidar_devices'] = devices
            STOP.wait(10.0)
    if JOURNAL is not None and JOURNAL.poll() is None:
        JOURNAL.terminate()
    emit('stop')


if __name__ == '__main__':
    main()
