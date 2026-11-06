#!/usr/bin/env bash
# Run on the laptop with mapping, navigation and teleop stopped; restarts robot bringup.
set -eo pipefail
asn_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -gt 0 ]]; then
  echo "Usage: $0 (run on laptop with robot connected and motion stopped)" >&2
  exit 2
fi
export ASN_CLOCK_SSH_SOCKET="${ASN_CLOCK_SSH_SOCKET:-/tmp/asn-clock-sync-ssh}"
echo 'Connecting to robot; SSH may request its password.'
ssh -o ConnectTimeout=8 -o ControlMaster=auto -o ControlPersist=120 -S "$ASN_CLOCK_SSH_SOCKET" ubuntu@192.168.77.1 true
python3 "$asn_root/scripts/sync_hardware_time.py"
source "$asn_root/ros2_ws/hardware_env.bash"
echo 'Checking lidar, odometry, RGB, depth and camera calibration from the laptop...'
if python3 "$asn_root/scripts/check_hardware_time.py" --camera --timeout 45; then
  exit 0
fi
echo 'Some streams are unavailable or stale. Requesting lidar and camera startup (no wheel motion)...'
# Bringup may stop sensors automatically while the robot is docked.
timeout 30 ros2 service call /start_motor std_srvs/srv/Empty '{}' || echo 'Lidar start request failed or timed out.' >&2
timeout 30 ros2 service call /oakd/start_camera std_srvs/srv/Trigger '{}' || echo 'Camera start request failed or timed out.' >&2
echo 'Rechecking all streams from the laptop (up to 90 seconds)...'
python3 "$asn_root/scripts/check_hardware_time.py" --camera --timeout 90
