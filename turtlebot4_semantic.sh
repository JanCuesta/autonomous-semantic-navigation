#!/usr/bin/env bash
# Laptop semantic stack supervisor. Never arms autonomy or sends goals.
set -eo pipefail
asn_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$asn_root/ros2_ws/hardware_env.bash"
asn_mode="${1:-full}"
if [[ $# -gt 0 ]]; then shift; fi
case "$asn_mode" in full|perception) ;; *) echo "Usage: $0 [full|perception] [map.yaml]" >&2; exit 2;; esac
asn_map="${1:-$asn_root/ros2_ws/maps/lab_remap.yaml}"
if [[ "$asn_mode" == full && ! -f "$asn_map" ]]; then echo "Missing map: $asn_map" >&2; exit 2; fi
# Refuse duplicate local project pipelines instead of starting competing nodes.
python3 - <<'PY'
from pathlib import Path
needles = ['asn_semantical_navigation.yolo_detector_node', '/asn_semantical_navigation/search_command_node', '__node:=hardware_depth_cloud', '__node:=coverage_depth_resize', '__node:=asn_nav2_container']
for p in Path('/proc').glob('[0-9]*/cmdline'):
    try: cmd = p.read_bytes().replace(b'\0', b' ').decode()
    except (OSError, UnicodeError): continue
    if any(x in cmd for x in needles):
        raise SystemExit('Project nodes already running (PID '+p.parent.name+'). Stop their launch terminal before using this supervisor.')
PY
python3 "$asn_root/scripts/check_hardware_time.py"
asn_pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${asn_pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
    for pid in "${asn_pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
ros2 launch asn_bringup hardware_sensors.launch.py &
asn_pids+=("$!")
if [[ "$asn_mode" == full ]]; then
    ros2 launch asn_bringup navigation.launch.py hardware:=true "map:=$(realpath "$asn_map")" use_rviz:=false &
    asn_pids+=("$!")
    ros2 launch asn_bringup semantical_navigation_hardware.launch.py enable_full_stack:=true &
else
    ros2 launch asn_bringup semantical_navigation_hardware.launch.py enable_perception:=true &
fi
asn_pids+=("$!")
echo "Semantic stack started disarmed. Set 2D Pose Estimate in full mode. No mission has been started."
wait -n "${asn_pids[@]}"
