#!/usr/bin/env bash
# Engage Create 3 software E-stop. This does not cancel Nav2 goals or release the stop.
set -euo pipefail
ssh -o ConnectTimeout=5 -o ControlMaster=auto -o ControlPersist=120 -S "${ASN_CLOCK_SSH_SOCKET:-/tmp/asn-clock-sync-ssh}" ubuntu@192.168.77.1 'bash -s' <<'REMOTE'
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_SUPER_CLIENT=TRUE
python3 - <<'PY'
import time
import rclpy
from irobot_create_msgs.srv import EStop
from irobot_create_msgs.msg import WheelStatus
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('asn_hardware_estop')
client = node.create_client(EStop, '/_do_not_use/e_stop')
state = []
sub = node.create_subscription(WheelStatus, '/wheel_status', lambda msg: state.append(msg.wheels_enabled), qos_profile_sensor_data)
try:
    if not client.wait_for_service(timeout_sec=12.0):
        raise RuntimeError('Create 3 E-stop service unavailable; stop is not confirmed')
    request = EStop.Request()
    request.e_stop_on = True
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    if not future.done() or future.exception() or not future.result().success:
        raise RuntimeError('Create 3 did not confirm E-stop; inspect the robot immediately')
    print('Create 3 confirmed E-stop:', future.result().message, flush=True)
    # Discard earlier status samples; require a fresh observation after the reply.
    state.clear()
    deadline = time.monotonic() + 8.0
    while not state and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not state or state[-1]:
        raise RuntimeError('Wheels-disabled status not confirmed; inspect the robot immediately')
    print('Wheel status confirms drive wheels disabled.', flush=True)
    print('Stop remains engaged. Cancel active goals before any release.', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
REMOTE
