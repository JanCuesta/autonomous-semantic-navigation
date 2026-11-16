#!/usr/bin/env python3
"""Read-only, bounded TurtleBot laboratory diagnostics; never commands motion."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
ROBOT = '192.168.77.1'

def run(command, limit=8):
    """Bound a diagnostic and reap its own descendants on timeout/interruption."""
    try:
        process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True,
                                   start_new_session=True)
    except OSError as exc:
        return 127, str(exc)
    try:
        out, err = process.communicate(timeout=limit)
        return process.returncode, (out + err).strip()
    except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
        # This group was created above for this diagnostic, never a user launch.
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            out, err = process.communicate(timeout=0.5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            out, err = process.communicate()
        if isinstance(exc, KeyboardInterrupt):
            raise
        return 124, 'TIMEOUT after %ss\n%s' % (limit, (out + err).strip())

def show(label, cmd, limit=8, fail='UNKNOWN', require_output=False):
    rc, output = run(cmd, limit)
    if rc == 0 and require_output and not output.strip():
        rc, output = 1, 'No output received; observation is UNKNOWN.'
    print('[%s] %s%s' % ('PASS' if rc == 0 else fail, label, '' if rc == 0 else ' (exit %s)' % rc), flush=True)
    if output:
        print(output[:5000], flush=True)
    return rc

def connection():
    print('Connection layers; SSH BatchMode never asks for credentials. UNKNOWN is not a pass.')
    show('route to robot', ['ip', 'route', 'get', ROBOT], 3, 'FAIL')
    show('ping', ['ping', '-c', '2', '-W', '2', ROBOT], 6, 'FAIL')
    print('ROS environment: ' + ' '.join('%s=%s' % (v, os.environ.get(v, '<unset>')) for v in ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','ROS_DISCOVERY_SERVER','ROS_SUPER_CLIENT')))
    show('SSH authenticated check; UNKNOWN may mean password/key needed', ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'ubuntu@'+ROBOT, 'hostname && systemctl is-active turtlebot4.service && pgrep -af "[f]astdds.*discovery|[d]iscovery.*server"'], 8)
    show('ROS graph; node list alone does not establish sensor delivery', ['ros2', 'node', 'list'], 10, require_output=True)

def process_rows():
    uid = os.getuid()
    needles = ('turtlebot4_semantic.sh', 'hardware_sensors.launch.py', 'semantical_navigation_hardware.launch.py', 'navigation.launch.py', 'hardware_depth_decode', 'coverage_depth_resize', 'hardware_depth_cloud', 'asn_nav2_container', 'asn_semantical_navigation', 'hardware_semantic.rviz')
    for p in sorted(Path('/proc').glob('[0-9]*'), key=lambda x:int(x.name)):
        try:
            if p.stat().st_uid != uid or int(p.name) == os.getpid(): continue
            cmd = p.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').strip()
            if not cmd or not any(n in cmd for n in needles): continue
            stat = p.joinpath('stat').read_text().split(') ',1)[1].split()
            yield p.name, stat[1], stat[2], cmd
        except (OSError, ValueError, IndexError): continue

def processes():
    print('Local same-user matches only; review command, PPID and PGID. No processes are signalled.')
    rows = list(process_rows())
    if not rows: print('No matching processes found; does not prove all launch children are gone.')
    for pid, ppid, pgid, cmd in rows: print('PID=%s PPID=%s PGID=%s %s' % (pid, ppid, pgid, cmd))

def preflight():
    print('Sensor freshness checks are read-only; PASS does not certify safe motion.')
    check = ROOT/'scripts/check_hardware_time.py'
    sensor_rc = show('scan, odom, RGB, depth, calibration freshness', [sys.executable, str(check), '--camera', '--timeout', '20'], 25, 'FAIL')
    results = [sensor_rc]
    for a,b in [('odom','base_link'),('base_link','rplidar_link')]:
        rc,out=run(['timeout','5','ros2','run','tf2_ros','tf2_echo',a,b],7)
        ok=rc in (0, 124) and 'Translation:' in out and 'Rotation:' in out
        results.append(0 if ok else 1)
        print('[%s] TF %s -> %s\n%s' % ('PASS' if ok else 'UNKNOWN',a,b,out[:1200]))
    results.append(show('velocity publishers and subscribers', ['ros2','topic','info','/cmd_vel','-v'], 8, require_output=True))
    results.append(show('battery sample', ['ros2','topic','echo','/battery_state','--once','--qos-reliability','best_effort'], 8, require_output=True))
    print('[UNKNOWN] Physical clearance, stop availability, map alignment, wheel condition and action ownership require operator checks.')
    return 1 if any(results) else 0

def monitor(duration):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String, Bool
        from geometry_msgs.msg import PoseStamped
        from action_msgs.msg import GoalStatusArray
    except ImportError as e: print('Source ros2_ws/hardware_env.bash first:',e); return 1
    rclpy.init(); node=Node('asn_lab_monitor_'+str(os.getpid()))
    values={}; subs=[]
    specs=[('mission',String,'/semantic/mission/state',True),('mission_detail',String,'/semantic/mission/status_json',True),('target_class',String,'/semantic/search/target_class',True),('detections',String,'/semantic/search/detections_json',False),('localization',String,'/semantic/perception/status',False),('target_valid',Bool,'/semantic/perception/target_valid',False),('target_pose',PoseStamped,'/semantic/perception/target_pose',False),('approach_status',String,'/semantic/approach/status',False),('approach_valid',Bool,'/semantic/approach/goal_valid',False),('approach_goal',PoseStamped,'/semantic/approach/goal',False),('nbv',String,'/semantic/nbv/execution_status',False),('nav_status',GoalStatusArray,'/navigate_to_pose/_action/status',True)]
    latch=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def callback(key,msg):
        if hasattr(msg,'data'): value=msg.data
        elif hasattr(msg,'pose'):
            p=msg.pose.position; value={'frame':msg.header.frame_id,'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,'x':round(p.x,3),'y':round(p.y,3),'z':round(p.z,3)}
        else: value=[int(x.status) for x in msg.status_list]
        values[key]=(time.monotonic(),value)
    for name,typ,topic,transient in specs: subs.append(node.create_subscription(typ,topic,lambda m,k=name: callback(k,m),latch if transient else 10))
    start=time.monotonic(); next_print=start+min(3,duration)
    try:
        while time.monotonic()-start < duration:
            rclpy.spin_once(node,timeout_sec=0.2)
            now=time.monotonic()
            if now>=next_print or now-start>=duration:
                print('\n--- observed values (age since receipt; UNKNOWN means no message) ---',flush=True)
                for key,*_ in specs:
                    print('%s: %s' % (key, ('%s [%.1fs ago]'%(json.dumps(values[key][1]),now-values[key][0])) if key in values else 'UNKNOWN'),flush=True)
                next_print=now+5
        print('\n--- final observed values (age since receipt) ---', flush=True)
        for key,*_ in specs:
            print('%s: %s' % (key, ('%s [%.1fs ago]' % (json.dumps(values[key][1]), time.monotonic()-values[key][0])) if key in values else 'UNKNOWN'), flush=True)
    finally: node.destroy_node(); rclpy.shutdown()
    return 0

def depth_comparison(source, decoded, start, end, margin=1.0):
    """Match nanosecond stamps inside both streams' guarded capture overlap."""
    if not source or not decoded or end - start <= 2 * margin:
        return None
    low = max(min(x[0] for x in source), min(x[0] for x in decoded)) + int(margin * 1e9)
    high = min(max(x[0] for x in source), max(x[0] for x in decoded)) - int(margin * 1e9)
    if low >= high:
        return None
    def select(rows):
        return {stamp: arrival for stamp, arrival, monotonic in rows
                if low <= stamp <= high and start + margin <= monotonic <= end - margin}
    src, dst = select(source), select(decoded)
    all_src = {stamp: arrival for stamp, arrival, _ in source}
    all_dst = {stamp: arrival for stamp, arrival, _ in decoded}
    matched = src.keys() & all_dst.keys()
    return src, dst, matched, all_src, all_dst


def timing(duration):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, Image
    except ImportError as e: print('Source ros2_ws/hardware_env.bash first:', e); return 1
    rclpy.init(); node = Node('asn_lab_rgbd_timing_' + str(os.getpid()))
    data = {'rgb': [], 'depth': [], 'compressed_depth': []}
    def cb(key, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        data[key].append((stamp, time.time(), time.monotonic()))
    subs = [node.create_subscription(kind, topic, lambda m, k=key: cb(k, m), qos_profile_sensor_data)
            for key, kind, topic in [
                ('rgb', CompressedImage, '/oakd/rgb/image_raw/compressed'),
                ('compressed_depth', CompressedImage, '/oakd/stereo/image_raw/compressedDepth'),
                ('depth', Image, '/hardware_depth/image_raw')]]
    start = time.monotonic(); end = start + duration
    try:
        while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=0.1)
    finally: node.destroy_node(); rclpy.shutdown()
    for key, rows in data.items():
        ages = [arrival - stamp / 1e9 for stamp, arrival, _ in rows]
        print('%s: %d samples, %.2f Hz over %.1fs; capture-to-arrival age %s' %
              (key, len(rows), len(rows)/duration, duration,
               ('%.3f to %.3fs' % (min(ages), max(ages))) if ages else 'UNKNOWN'))
        stamps = sorted({row[0] for row in rows})
        gaps = [(b-a)/1e9 for a,b in zip(stamps, stamps[1:])]
        print('%s capture timestamp gaps: %s; duplicate stamps: %d' %
              (key, ('min %.3f, max %.3fs' % (min(gaps), max(gaps))) if gaps else 'UNKNOWN', len(rows)-len(stamps)))
    if data['rgb'] and data['depth']:
        gaps = sorted(min(abs(s[0]-d[0]) / 1e9 for d in data['depth']) for s in data['rgb'])
        print('Nearest received depth per RGB header: median %.3fs, max %.3fs; %d/%d > 0.10s' %
              (gaps[len(gaps)//2], gaps[-1], sum(x > 0.10 for x in gaps), len(gaps)))
    else:
        print('RGB/depth comparison: UNKNOWN; both streams required.')
    comparison = depth_comparison(data['compressed_depth'], data['depth'], start, end)
    if comparison is None:
        print('Compressed/decoded match: UNKNOWN; need both streams and overlapping timestamps after 1s boundary exclusion.')
    else:
        src, dst, matched, all_src, all_dst = comparison
        print('Interior exact-stamp comparison (1s excluded at arrival and shared capture boundaries):')
        print('source unique: %d; decoded unique: %d; matched: %d; source missing decoded: %d; decoded missing source: %d' %
              (len(src), len(dst), len(matched), len(src.keys()-all_dst.keys()), len(dst.keys()-all_src.keys())))
        delays = sorted(all_dst[k]-src[k] for k in matched)
        print('Matched decoded arrival minus compressed arrival: %s' %
              (('min %.3f, median %.3f, max %.3fs' % (delays[0], delays[len(delays)//2], delays[-1])) if delays else 'UNKNOWN'))
        print('Counterparts are matched across the full window; 1s guards reduce boundary errors but cannot exclude delays beyond the guard. Missing counts are observations, not proven decoder loss.')
    print('RGB/depth nearest-pair diagnostic is not identical to the localizer buffer; arrival age includes clock error, transport and scheduling.')
    return 0 if all(data.values()) and comparison is not None else 1

def evidence():
    base=ROOT/'results'/'lab_evidence'; base.mkdir(parents=True,exist_ok=True)
    folder=base/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')); folder.mkdir()
    tasks=[('system',['uname','-a']),('route',['ip','route','get',ROBOT]),('git',['git','-C',str(ROOT),'status','--short']),('nodes',['ros2','node','list']),('topics',['ros2','topic','list','-t']),('actions',['ros2','action','list','-t']),('cmd_vel',['ros2','topic','info','/cmd_vel','-v']),('nav_status',['ros2','topic','echo','/navigate_to_pose/_action/status','--once','--qos-durability','transient_local']),('detector_parameter',['ros2','param','get','/yolo_detector_node','confidence_threshold']),('localizer_parameter',['ros2','param','get','/object_localizer_node','max_sync_difference_s'])]
    for name,cmd in tasks:
        rc,out=run(cmd,7); (folder/(name+'.txt')).write_text('COMMAND: '+' '.join(cmd)+'\nEXIT: '+str(rc)+'\n'+out+'\n'); print('%s: %s'%(name,'OK' if rc==0 else 'UNKNOWN/FAILED'))
    (folder/'processes.txt').write_text('\n'.join('PID=%s PPID=%s PGID=%s %s'%x for x in process_rows())+'\n')
    (folder/'environment.txt').write_text('\n'.join('%s=%s'%(v,os.environ.get(v,'<unset>')) for v in ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','ROS_DISCOVERY_SERVER','ROS_SUPER_CLIENT'))+'\n')
    # One shared ROS subscriber samples all semantic topics concurrently.
    rc, out = run([sys.executable, str(Path(__file__).resolve()),
                   'monitor', '--seconds', '10'], 15)
    (folder/'semantic_samples.txt').write_text('EXIT: %s\n%s\n' % (rc, out))
    log = Path.home()/'asn-lab-launch.log'
    try:
        # Read only the bounded tail, even for a very large launch log.
        with log.open('rb') as handle:
            handle.seek(0, 2)
            handle.seek(max(0, handle.tell()-65536))
            tail = handle.read(65536).decode(errors='replace')
        (folder/'launch_log_tail.txt').write_text('SOURCE: %s (last <=64 KiB)\n%s' % (log, tail))
    except OSError as exc:
        (folder/'launch_log_tail.txt').write_text('UNKNOWN: %s\n' % exc)
    print('Evidence saved:', folder, '; includes semantic samples and bounded launch-log tail. Files can contain local paths and network details.')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    s=p.add_subparsers(dest='mode',required=True)
    for name,helptext in [('connection','network, SSH and DDS layers'),('preflight','freshness, basic TF, command interface and battery'),('processes','list relevant same-user local processes only'),('evidence','save bounded diagnostic samples')]: s.add_parser(name,help=helptext)
    for name,helptext in [('monitor','display mission and perception observations'),('timing','compare received RGB/depth header stamps')]:
        a=s.add_parser(name,help=helptext); a.add_argument('--seconds',type=float,default=20,help='observation duration (1–300 seconds)')
    args=p.parse_args()
    if hasattr(args,'seconds') and not 1<=args.seconds<=300: p.error('--seconds must be 1–300')
    if args.mode=='connection': connection()
    elif args.mode=='processes': processes()
    elif args.mode=='preflight': sys.exit(preflight())
    elif args.mode=='evidence': evidence()
    elif args.mode=='monitor': sys.exit(monitor(args.seconds))
    elif args.mode=='timing': sys.exit(timing(args.seconds))

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted; diagnostics stopped. No robot stop was requested.", file=sys.stderr)
        sys.exit(130)
