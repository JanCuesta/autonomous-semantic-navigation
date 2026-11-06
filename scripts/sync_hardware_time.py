"""Synchronize Pi and Create 3 clocks over an authenticated SSH master."""
import datetime
import os
import shlex
import subprocess
import time

ssh = ['ssh', '-S', os.environ['ASN_CLOCK_SSH_SOCKET'], 'ubuntu@192.168.77.1']
def remote(command):
    return subprocess.run(ssh + [command], check=True, capture_output=True, text=True).stdout.strip()

def offset_samples():
    samples = []
    for _ in range(3):
        start = time.time()
        robot = float(remote('date +%s.%N'))
        finish = time.time()
        samples.append((finish - start, robot - (start + finish) / 2))
    return min(samples)

remote('sudo -n systemctl stop turtlebot4.service')
try:
    # Measure SSH transport first; compute the new timestamp after shutdown.
    for attempt in range(3):
        rtt, offset = offset_samples()
        if abs(offset) < 0.08:
            break
        remote('sudo -n date -u -s @' + str(time.time() + rtt / 2))
    rtt, offset = offset_samples()
    print(f'Pi minus laptop clock: {offset:+.3f}s (best SSH RTT {rtt:.3f}s)', flush=True)
    if abs(offset) > 0.25:
        raise RuntimeError('Clock correction could not be verified; do not navigate')
    base_sync = """import datetime,urllib.request,urllib.parse,time,math
point=math.ceil(time.time()+0.5)
stamp=datetime.datetime.fromtimestamp(point,datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%S')
base='http://192.168.186.2'
time.sleep(max(0,point-time.time()-0.01))
urllib.request.urlopen(base+'/beta-datetime-save',data=urllib.parse.urlencode({'newdatetime':stamp}).encode(),timeout=10).read()
urllib.request.urlopen(urllib.request.Request(base+'/api/restart-ntpd',data=b'',method='POST'),timeout=10).read()
print('Create 3 time set; NTP resynchronization requested')
"""
    print(remote('python3 -c ' + shlex.quote(base_sync)), flush=True)
finally:
    remote('sudo -n systemctl start turtlebot4.service')
