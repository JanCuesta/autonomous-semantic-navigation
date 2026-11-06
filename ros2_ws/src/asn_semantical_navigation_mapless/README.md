# Autonomous Semantic Navigation — truly mapless prototype

This is a **separate copy** of the existing `asn_semantical_navigation` ROS 2 package. It retains the original YOLO detector, text command parser, RGB-D 3D object localization, semantic mission state machine, mission cancellation, and approach handoff. It removes known-map occupancy coverage, map-based NBV/frontier search, AMCL, and global Nav2 route planning.

## What changed

| Existing module | Mapless modification |
|---|---|
| `search_command_node`, `search_commands`, `target_filter`, `yolo_detector_node`, UI | Copied unchanged |
| `object_localizer_node` | Destination frame is `odom` instead of `map` |
| `semantical_mission_manager_node` | Same mission state machine, odom-frame goals, same ROS interfaces |
| `nbv_auto_node` | Live LiDAR sector clearance + novelty of last 100 odom viewpoints; no grid/map |
| `approach_goal_node` | Live LiDAR checked radial standoff candidates, odom target |
| `mapless_motion_node` | New local action server compatible with `NavigateToPose`, *not* Nav2; reactive turn/drive/turn, live LiDAR stop checks, cancellation, sensor expiry |

**Important:** `odom` is a drifting relative reference, **not a persistent map**. Short-term remembered poses/headings are used only to discourage recent repeated views. No occupancy grid is built. Therefore, global coverage, global shortest paths, and complete exploration are not guaranteed. Our controller aborts when its immediate route is blocked; it does not solve obstacle detours. This is an experimental local-reactive baseline.

## Install as a copy of your existing package

From your Ubuntu machine, with your existing repository checked out:

```bash
# Extract mapless_semantic_patch.zip somewhere, then:
python3 /path/to/mapless_semantic_patch/install_mapless.py \
  --repo ~/projects/autonomous-semantic-navigation

cd ~/projects/autonomous-semantic-navigation/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select asn_semantical_navigation_mapless
source install/setup.bash
colcon test --packages-select asn_semantical_navigation_mapless
colcon test-result --verbose
```

If the destination package already exists, the installer stops instead of overwriting it. The original package and its launch files are unchanged.

## Safe bringup / validate first with NO motion

Launch your existing Isaac Sim and ROS bridge/robot driver separately. **Do not launch the old semantic stack or Nav2 concurrently**, because the old system also owns `/semantic/*` topics and may publish velocity commands. The mapless launch intentionally launches no map server, SLAM, AMCL, Nav2, or costmap.

```bash
ros2 launch asn_semantical_navigation_mapless mapless_semantic.launch.py \
  use_sim_time:=true enable_motion:=false scan_topic:=/scan

# Second sourced terminal:
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link
ros2 action list -t
ros2 topic info /mapless/cmd_vel
```

Motion is disabled by default. Velocity commands go to an **isolated** `/mapless/cmd_vel` topic by default; they will not drive the robot until you deliberately connect that topic to a correctly typed, watchdog-protected actuator interface. The default command message is `geometry_msgs/msg/TwistStamped`; adjust `twist_stamped:=false` only if your simulator actually requires `Twist`.

For **supervised simulation after safety checks**, launch with `enable_motion:=true` and explicitly select an appropriate simulated command topic, e.g. `cmd_vel_topic:=/cmd_vel`, **only after verifying it has the matching TwistStamped type and exactly one intended command publisher**. Do not use this step on the physical robot before an independent validated stop chain.

The copied YOLO detector is optional to launch with `start_yolo:=true` (requires working `perception_python`, topics and your existing environment). Otherwise start the detector independently. Set `rgb_topic`, `depth_topic`, and `camera_info_topic` for your current camera configuration. Depth must be aligned with RGB and TF available.

## Mission commands

After starting the detector and observing fresh scans and camera data:

```bash
ros2 topic pub --once /semantic/search/command std_msgs/msg/String \
  "{data: 'find a chair'}"
ros2 service call /semantic/mission/start std_srvs/srv/Trigger '{}'

# Cancel and verify the motion action becomes idle:
ros2 service call /semantic/mission/cancel std_srvs/srv/Trigger '{}'
ros2 service call /semantic/nbv/idle_check std_srvs/srv/Trigger '{}'
ros2 topic echo /semantic/mission/status_json
```

No autonomous motion starts simply because the launch file was run: motion must be explicitly enabled **and** a target mission explicitly started. This software is **not** a hardware emergency stop; retain your TurtleBot physical/manual stop and base-level command expiry. No physical safety or end-to-end ROS integration claim is made until tested in Isaac Sim and controlled hardware trials.

## Validation checklist

1. Build + pytest unit tests; confirm the stack starts with no `/map` publisher or subscriber required.
2. Disable motion; confirm a goal is rejected and no commands reach `/cmd_vel`.
3. In Isaac Sim only: command a short clear move; confirm odom progress, tolerances, and final stop.
4. Put an obstacle in front of the robot, then remove LiDAR/TF; verify abort and zero velocity in both cases.
5. Verify goal cancellation, mission shutdown, YOLO-loss timeout, RGB-D target locking, and approach offset.
6. Check the mapless action server and original Nav2 cannot concurrently own robot velocity.
7. Compare object-search success, distance, latency, repeated viewpoints, collisions, aborts to the original mapped baseline, on several unseen layouts.

### Known limitations

- Purely local obstacle avoidance: blocked straight-line segments abort rather than route around walls.
- Short-term odom memory drifts; places can be revisited, search completeness is not guaranteed.
- Only one mapless motion action at a time; no recovery while mapless controller is blocked.
- Existing mission manager's Nav2 action references are compatibility interfaces to the **new** local action server, not claims that Nav2 is running.
- RGB-D and LiDAR calibration, sensor QoS, clock synchronization, and the velocity interface must be validated in your specific sim/hardware setup.
- No source or unit tests here prove collision-free real-world performance.
