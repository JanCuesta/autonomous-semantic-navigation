# Semantic navigation on the lab TurtleBot 4

The lab project's published hardware interface uses the `robot_09` namespace
and an OAK-D RGB `sensor_msgs/msg/CompressedImage` stream at
`/robot_09/oakd/rgb/image_raw/compressed`. This project has a separate
`semantical_navigation_hardware.launch.py` entry point. It does not start
Isaac Sim, a robot driver, SLAM, or Nav2. Bring up the lab's existing robot and
navigation stack separately, with exactly one map-to-odom TF publisher.

## 1. Read-only discovery before any motion

On a computer connected to the lab ROS network, source ROS 2 and the workspace.
Use the lab's actual `ROS_DOMAIN_ID`, DDS, and discovery setup. Inspect:

```bash
ros2 topic list -t | grep /robot_09/
ros2 action list -t | grep navigate_to_pose
ros2 topic echo /robot_09/oakd/rgb/image_raw/compressed --once --field header
ros2 topic echo /robot_09/map --once --field header
ros2 run tf2_ros tf2_echo map base_link
```

Record the actual aligned depth `sensor_msgs/msg/Image` topic, its
`sensor_msgs/msg/CameraInfo` topic, a compatible `sensor_msgs/msg/PointCloud2`
topic, and the location of `/tf` and `/tf_static`. Inspect their types and
headers with `ros2 topic info -v` and `ros2 topic echo --once --field header`.
The object localizer requires RGB and depth images with matching optical
`frame_id`, width, height, and timestamps within 0.10 s. Depth must be
`16UC1` (millimeters) or `32FC1` (meters). A raw unaligned depth image is not
enough. The mission supervisor currently requires TF IDs `map` and
`base_link`; a robot namespace does not alter frame IDs.

## 2. RGB-only integration

Build and source this workspace. Ensure `perception_python` points to an
environment with this Python package, `ultralytics`, OpenCV, and `cv_bridge`.
The default is the same perception virtual environment used by the simulation
launch. Run:

```bash
ros2 launch asn_bringup semantical_navigation_hardware.launch.py \
  robot_namespace:=robot_09
```

This launches the target selector and YOLO only. It accepts the lab's
compressed RGB topic directly and publishes project-local `/semantic/...`
detections. It cannot navigate in this mode. Check a chosen class and the
detection stream before enabling the full stack:

```bash
ros2 topic pub --once /semantic/search/command std_msgs/msg/String \
  "{data: 'find a backpack'}"
ros2 topic echo /semantic/search/detections_json
```

Only COCO classes available in the selected YOLO model can be searched.
The lab project's ArUco follower uses a different perception pipeline; its
board detections cannot be substituted for these YOLO detections.

## 3. Full stack after interface validation

Use the topic names observed in the read-only checks, not guessed OAK-D
defaults:

```bash
ros2 launch asn_bringup semantical_navigation_hardware.launch.py \
  robot_namespace:=robot_09 enable_full_stack:=true \
  depth_topic:=/robot_09/VERIFIED_ALIGNED_DEPTH_IMAGE \
  camera_info_topic:=/robot_09/VERIFIED_ALIGNED_CAMERA_INFO \
  pointcloud_topic:=/robot_09/VERIFIED_POINTCLOUD
```

The launch checks that all three topic arguments are supplied; it cannot
verify their type or spatial registration without the live robot. It remaps
robot-facing map, Nav2 action/status, speed-limit, and TF endpoints to
`/robot_09/...`, while preserving internal `/semantic/...` interfaces.
If the lab publishes TF globally, override `tf_topic:=/tf` and
`tf_static_topic:=/tf_static`. If the actual frames differ from `map` and
`base_link`, adapt the mission manager and its pose checks before enabling
the full stack.

The full stack starts with NBV disarmed and the mission manager idle. Check
`/semantic/perception/status`, `/semantic/nbv/status`, the coverage map,
Nav2 action availability, and the localizer's RGB/depth validation first.
During a supervised lab run, select a target and explicitly call
`/semantic/mission/start`. Use `/semantic/mission/cancel` to cancel the
mission. Keep other Nav2 goal clients and the lab's direct `/robot_09/cmd_vel`
follower inactive during this run. Review robot footprint, speed limits,
collision protection, and a human stop procedure before motion.

## Remaining hardware-dependent work

- The lab's RGB topic is documented; aligned depth, camera calibration,
  PointCloud2, TF, and Nav2 action names must be observed on the robot.
- If the camera publishes depth in a different optical frame or resolution,
  configure aligned depth on the OAK-D rather than weakening the localizer's
  frame and timestamp checks.
- If there is no compatible PointCloud2 stream, add a registered point-cloud
  source or adapt `coverage_node.py`; NBV measured coverage cannot work
  without it.
- This is launch and interface preparation. No hardware integration or
  motion acceptance is claimed until the physical checks pass.
