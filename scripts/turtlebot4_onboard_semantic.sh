#!/usr/bin/env bash
# Run on TurtleBot Pi. Headless full semantic stack; never arms autonomy.
set -eo pipefail
asn_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
asn_map="${1:-$asn_root/ros2_ws/maps/lab_20261113_173935.yaml}"
if [[ $# -gt 1 || ! -f "$asn_map" ]]; then
  echo "Usage: $0 [absolute-map.yaml]; map must exist" >&2; exit 2
fi
if [[ ! -f /etc/turtlebot4/setup.bash || ! -f "$asn_root/ros2_ws/install/setup.bash" ]]; then
  echo 'Robot ROS setup or onboard workspace install is missing.' >&2; exit 2
fi
source /etc/turtlebot4/setup.bash
source "$asn_root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_SUPER_CLIENT=TRUE
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/turtlebot4/fastdds_rpi.xml
asn_python="$asn_root/.venv-perception/bin/python"
if [[ ! -x "$asn_python" ]]; then echo 'Onboard perception venv missing.' >&2; exit 2; fi
"$asn_python" -c 'import cv_bridge, ultralytics, torch' || { echo 'Onboard perception dependencies missing.' >&2; exit 2; }
python3 - <<'PY'
from pathlib import Path
needles = ('asn_semantical_navigation.yolo_detector_node', '/asn_semantical_navigation/search_command_node', '__node:=hardware_depth_cloud', '__node:=coverage_depth_resize', '__node:=asn_nav2_container')
for p in Path('/proc').glob('[0-9]*/cmdline'):
    try: cmd=p.read_bytes().replace(b'\0',b' ').decode()
    except (OSError, UnicodeError): continue
    if any(s in cmd for s in needles):
        raise SystemExit('Existing local project node PID '+p.parent.name+'; stop its owning launch first.')
PY
python3 "$asn_root/scripts/check_hardware_time.py" --camera --timeout 45
asn_pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${asn_pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${asn_pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
ros2 launch asn_bringup hardware_sensors.launch.py & asn_pids+=("$!")
ros2 launch asn_bringup navigation.launch.py hardware:=true "map:=$(realpath "$asn_map")" use_rviz:=false & asn_pids+=("$!")
ros2 launch asn_bringup semantical_navigation_hardware.launch.py enable_full_stack:=true use_rviz:=false "perception_python:=$asn_python" & asn_pids+=("$!")
echo 'Onboard semantic stack started disarmed; no goal sent. Verify map pose and operator stop before motion.'
wait -n "${asn_pids[@]}"
