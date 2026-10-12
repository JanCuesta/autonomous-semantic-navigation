#!/usr/bin/env bash
# Read-only ROS 2 connectivity and interface check for the TurtleBot 4 laptop side.
# Source ROS 2 and the project workspace before running this script.
set -u

stamp=$(date +%Y%m%d_%H%M%S)
report="turtlebot4_hardware_check_${stamp}.txt"

{
  echo "TurtleBot 4 hardware interface check"
  echo "Date: $(date -Is)"
  echo "Host: $(hostname)"
  echo "ROS_DISTRO: ${ROS_DISTRO:-not set}"
  echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default/not set}"
  echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0 (default)}"
  echo

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 command not found. Source the ROS 2 setup.bash first."
    exit 1
  fi

  echo "===== DISCOVERED NODES ====="
  timeout 10s ros2 node list || echo "Node discovery timed out or failed. Check Wi-Fi/DDS settings."
  echo

  echo "===== DISCOVERED TOPICS AND TYPES ====="
  timeout 10s ros2 topic list -t || echo "Topic discovery timed out or failed."
  echo

  echo "===== KEY TOPIC PUBLISHERS ====="
  for topic in /scan /odom /joint_states /imu /tf /tf_static /cmd_vel; do
    echo "--- $topic ---"
    timeout 8s ros2 topic info "$topic" -v 2>&1 || true
  done
  echo

  echo "===== SENSOR SAMPLE CHECKS (up to 5 seconds each) ====="
  for topic in /scan /odom /imu; do
    echo "--- $topic ---"
    timeout 5s ros2 topic echo "$topic" --once 2>&1 || echo "No message received within 5 seconds (topic may be absent or use a different name)."
  done
  echo

  echo "===== TF FRAME GRAPH ====="
  if ros2 pkg prefix tf2_tools >/dev/null 2>&1; then
    timeout 20s ros2 run tf2_tools view_frames 2>&1 || echo "TF graph command failed or timed out."
    echo "Look for frames.pdf in the directory where the script was run."
  else
    echo "tf2_tools is not installed; install/use it later to generate frames.pdf."
  fi
  echo

  echo "===== QUICK TF CHECKS ====="
  for pair in "odom base_link" "map odom" "base_link laser_frame" "base_footprint base_link"; do
    read -r parent child <<< "$pair"
    echo "--- $parent -> $child (3 seconds) ---"
    timeout 3s ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true
  done
  echo
  echo "Check the discovered frame names against the hardware YAML before launching Nav2."
} 2>&1 | tee "$report"

echo
echo "Saved text report: $report"
