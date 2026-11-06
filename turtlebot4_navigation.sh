#!/usr/bin/env bash
# Laptop entry point; starting mapping/navigation does not send a goal.
set -eo pipefail
asn_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$asn_root/ros2_ws/hardware_env.bash"
case "${1:-}" in
  mapping)
    python3 "$asn_root/scripts/check_hardware_time.py"
    shift
    exec ros2 launch asn_bringup mapping.launch.py hardware:=true "$@"
    ;;
  navigation|localization)
    mode="$1"; shift
    if [[ $# -lt 1 || ! -f "$1" ]]; then
      echo "Usage: $0 $mode /absolute/path/to/saved_map.yaml [launch arguments]" >&2
      exit 2
    fi
    map_file="$(realpath -- "$1")"; shift
    python3 "$asn_root/scripts/check_hardware_time.py"
    exec ros2 launch asn_bringup "$mode.launch.py" hardware:=true "map:=$map_file" "$@"
    ;;
  save-map)
    shift
    if [[ $# -ne 1 ]]; then echo "Usage: $0 save-map /path/to/map_basename" >&2; exit 2; fi
    mkdir -p -- "$(dirname -- "$1")"
    exec ros2 run nav2_map_server map_saver_cli -f "$1" --ros-args -p save_map_timeout:=30.0
    ;;
  *) echo "Usage: $0 {mapping|navigation MAP.yaml|localization MAP.yaml|save-map MAP_BASENAME}" >&2; exit 2 ;;
esac
