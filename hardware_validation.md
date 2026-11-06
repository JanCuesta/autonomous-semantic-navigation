# TurtleBot 4 — Hardware Validation Report

**Robot:** `robot_05`  
**Environment:** ROS 2 Jazzy — Ubuntu 24.04  
**Repository:** `autonomous-semantic-navigation`  
**Commit:** `436e71e` — Hardware testing  
**Date:** November 5, 2026

## #1: Timestamp Synchronization

Created a script at:

`~/projects/autonomous-semantic-navigation/turtlebot4_sync_time.sh`

The script synchronizes the timestamps between the robot and the laptop.

To execute:

```bash
cd ~/projects/autonomous-semantic-navigation
./turtlebot4_sync_time.sh
```

**STATUS: VALIDATED**

## #2: LiDAR and Odometry Physical Testing

- Placed an object exactly 30 cm from the robot's front base edge.
- Measured the object distance using the LiDAR.
- Moved the robot to the object's original position without lifting it, allowing the wheels to rotate normally.
- Measured the resulting odometry displacement.

**STATUS: VALIDATED**

## #3: Wi-Fi and ROS 2 Communication

Confirmed communication between the laptop and TurtleBot through the laboratory Wi-Fi network.

Verified:
- ROS 2 discovery functioning.
- Robot nodes visible from the laptop.
- Fast DDS communication established.
- Hardware topics accessible.

Environment setup:

```bash
cd ~/projects/autonomous-semantic-navigation/ros2_ws
source hardware_env.bash
```

Configuration:
- `ROS_DOMAIN_ID=0`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_DISCOVERY_SERVER=192.168.77.1:11811`
- `ROS_SUPER_CLIENT=TRUE`

**STATUS: VALIDATED**

## #4: Robot Battery and Status

Checked battery using:

```bash
ros2 topic echo /battery_state --once
```

Results:
- Battery: 88%
- Voltage: 15.838 V
- Temperature: 40.65°C
- Battery present: True

**STATUS: VALIDATED**

## #5: LiDAR ROS 2 Data

Verified that the LiDAR publishes measurements through `/scan`.

Commands:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once --field header --qos-reliability best_effort
```

Results:
- Publishing frequency: approximately 7.6 Hz.
- Minimum range: 0.15 m.
- Maximum range: 12.0 m.
- Frame: `rplidar_link`.

**STATUS: VALIDATED — Data publishing and measurement range**

## #6: Odometry ROS 2 Data

Verified odometry measurements through `/odom`.

Commands:

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
```

Results:
- Publishing frequency: approximately 19.5 Hz.
- Frame: `odom`.
- Child frame: `base_link`.
- Linear and angular velocities approximately zero while stationary.
- Position and orientation data available.

Observed that the odometry covariance matrix contains only zeros. This should be reviewed before final navigation validation.

**STATUS: VALIDATED — Data publishing and stationary behavior**

## #7: TF Transform Validation

Confirmed the robot's coordinate-frame connections.

**Odometry to robot base:**

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Transform successfully received, with stable position and orientation while the robot remained stationary.

**Robot base to LiDAR:**

```bash
ros2 run tf2_ros tf2_echo base_link rplidar_link
```

Results:
- Translation: X = 0.004 m, Y = 0 m, Z = 0.139 m.
- Rotation: 90° yaw.
- Stable static transformation.

**STATUS: VALIDATED — TF availability**

## #8: Sensor Timestamp Validation

Compared the laptop system time with ROS sensor timestamps.

Commands:

```bash
date -u
date +%s

ros2 topic echo /scan --once --field header --qos-reliability best_effort
ros2 topic echo /odom --once --field header
```

Results:
- System time and sensor timestamps consistent.
- LiDAR and odometry use Unix timestamps.
- No major timestamp offset observed.

**STATUS: VALIDATED**

## #9: Motor Command Interface

Inspected the robot's movement-command topics.

Commands:

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_unstamped -v
ros2 node info /create3_repub
```

Results:

| Topic | Message type |
|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| `/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` |

The `create3_repub` node subscribes to both topics and communicates with the Create 3 hardware.

Additional findings:
- `/e_stop` service available.
- `/robot_power` service available.
- `teleop_twist_joy_node` publishes to `/cmd_vel`.
- The joystick is not being used for testing, but its publisher remains active.

**STATUS: INTERFACES VERIFIED — Command ownership and stopping behavior pending**

## #10: Saved Map and Navigation Files

Confirmed the latest GitHub commit and local map files.

```bash
git log -1 --oneline
ls -lh maps/my_environment.*
```

Results:
- Commit: `436e71e` — Hardware testing.
- `my_environment.pgm`: 101 KB.
- `my_environment.yaml`: 138 bytes.
- Map resolution: 0.05 m/pixel.

The hardware Nav2 launch configuration has also been identified.

**STATUS: FILES VERIFIED — Map alignment and navigation pending**

---

## Remaining Validations Before the First Autonomous Goal

- [ ] Verify manual/emergency stopping and controller command timeout.
- [ ] Prevent conflicting velocity-command publishers.
- [ ] Confirm the physical robot footprint and reduce initial navigation speed limits.
- [ ] Verify the saved map matches the laboratory environment.
- [ ] Launch the hardware Nav2 stack without commanding movement.
- [ ] Verify all required Nav2 lifecycle nodes are active.
- [ ] Validate AMCL localization and `map → odom` TF.
- [ ] Inspect map, LiDAR alignment, and costmaps in RViz.
- [ ] Generate and inspect a navigation path without executing it.
- [ ] Perform the final safety check before the first supervised autonomous goal.

**Current conclusion:** Network communication, sensor publishing, timestamps, essential TF connections, and command interfaces have been verified. Autonomous navigation and its physical safety mechanisms are not yet fully validated.



## #11: Physical Motion and Stopping

- A 30 cm manual movement produced approximately 28.4 cm of odometry displacement.
- A short powered forward test traveled approximately 4.9 cm with a 0.3° heading change.
- The Create 3 software emergency stop disabled the drive wheels.
- Wheel feedback returned to zero approximately 0.6 seconds after velocity commands stopped.
- The unused joystick publisher was stopped for the test session.

**STATUS: VALIDATED — Supervised short tests**

## #12: TF and Navigation Configuration

- Removed a conflicting second parent for `base_footprint`.
- Confirmed navigation uses `/scan`, `/odom`, and `base_link`.
- Set the costmap radius to 0.22 m and reduced the maximum forward speed to 0.05 m/s.
- Increased the collision monitor's transform wait while keeping collision monitoring and its sensor timeout enabled.
- Verified that the collision monitor stopped output when LiDAR data became stale.

**STATUS: VALIDATED — Configuration and stationary safety checks**

## #13: Laboratory Remapping

Remapped the laboratory using SLAM Toolbox and saved:

- `ros2_ws/maps/lab_remap.yaml`
- `ros2_ws/maps/lab_remap.pgm`

The map was loaded in RViz, and an initial pose was set using AMCL.

**STATUS: VALIDATED — Map created and loaded**

## #14: Normal Navigation

- Started the hardware Nav2 stack with `lab_remap`.
- Confirmed the required navigation and localization lifecycle nodes were active.
- Generated a short path without executing it.
- Sent a supervised goal from RViz. Nav2 reported success, and the operator confirmed that the robot reached the goal.

**STATUS: VALIDATED — One supervised autonomous goal**

## Remaining Issues

- The operator observed slight map misalignment.
- Intermittent LiDAR/TF timing warnings and occasional dropped scans remain under investigation.
- Additional goals and repeated runs are needed to assess navigation reliability.
- Semantic navigation has not yet been validated with the new map.

**Current conclusion:** Normal navigation reached one supervised goal on the
new laboratory map. Further testing is needed to improve map alignment and
confirm repeatable navigation and safety behavior.