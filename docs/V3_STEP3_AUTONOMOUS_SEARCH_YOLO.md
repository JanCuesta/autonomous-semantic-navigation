# V3 Step 3 — Autonomous NBV + confidence frontiers + YOLO + text target

**Development overlay on top of Steps 1 and 2. The original coverage node and Nav2 configuration remain untouched.**

## Exactly what this implements

1. `nbv_auto_node` reuses the Step 2 one-action `NavigateToPose` client, the 3×3 grid of map-frame ±0.6 m offsets, the ±45°/±90° rotation candidates, and confidence-gain scoring. It can automatically select the next action **after** a successful, pose-validated previous action. No new goal is sent before the prior result settles.
2. When the local planner is exhausted, an initial **low-confidence frontier fallback** samples safe navigable positions from the live occupancy map near low-CF regions (`C < 30` initially). It chooses a predicted useful viewpoint; Nav2 still owns collision checking and execution. This is a simple initial method, not guaranteed complete search.
3. `search_command_node` parses a text request such as `find a backpack` into a supported COCO object class. This is a **deterministic text parser, not a language model**. It doesn't understand arbitrary requests, attributes or synonyms beyond its explicit list; `black` is ignored by YOLO category detection.
4. `yolo_detector_node` subscribes to a **real ROS2 sensor_msgs/Image** stream (not `/depth_pcl`), runs a YOLO model, requires 3 consecutive frames containing the requested class above a configurable confidence, and publishes `/semantic/nbv/target_found`. This is 2D detection only: **no 3D target localization, object-approach goal or semantic room classification yet**.
5. A detected target automatically disarms auto-navigation and requests cancellation of the active Nav2 goal. The robot only moves after an **explicit** `/semantic/nbv/auto_enable` request (`std_srvs/SetBool`, true).

## Install overlay

```bash
cd ~/projects/autonomous-semantic-navigation
unzip -o ~/Downloads/v3_nbv_step3_autonomy_overlay.zip -d .
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select asn_semantical_navigation
source install/setup.bash
```

Stop the old `nbv_execute_once_node` before starting `nbv_auto_node` because they expose the same manual `execute_once` and `cancel` services. Continue using the existing Isaac Sim safe launcher. Start Nav2, localization, `/semantic/coverage_map` and RViz as in Step 2.

## Test A — autonomous coverage only (before YOLO)

First test just **2 actions** while supervising the simulation, without requiring a detector:

```bash
ros2 run asn_semantical_navigation nbv_auto_node --ros-args -p use_sim_time:=true -p require_target:=false -p require_live_detector:=false -p auto_max_goals:=2
```

In another sourced terminal, arm:

```bash
ros2 service call /semantic/nbv/auto_enable std_srvs/srv/SetBool '{data: true}'
```

Watch `/semantic/nbv/execution_status` and Nav2 / RViz. It stops automatically at the goal budget or immediately if an action times out, Nav2 aborts, pose check fails or measured coverage gain repeatedly remains negligible. To interrupt before the action finishes:

```bash
ros2 service call /semantic/nbv/auto_enable std_srvs/srv/SetBool '{data: false}'
```

It requests Nav2 cancellation; verify the robot actually stops. There is no claim this is a hardware-certified E-stop.

## Test B — find the actual RGB image topic

```bash
ros2 topic list -t | grep 'sensor_msgs/msg/Image'
```

The existing repository documents `/depth_pcl` for coverage, **but does not establish that an RGB Image topic is published**. If the command prints nothing, configure an RGB camera ROS 2 publisher in Isaac Sim first. Do not guess the camera topic or substitute a PointCloud2 as an RGB image.

## Test C — optional YOLO setup

On Ubuntu 24.04, install `ros-jazzy-cv-bridge` via apt. A venv with ROS system packages is preferable for Ultralytics (avoid `--break-system-packages`):

```bash
sudo apt install ros-jazzy-cv-bridge
cd ~/projects/autonomous-semantic-navigation
python3 -m venv --system-site-packages .venv-yolo
source .venv-yolo/bin/activate
python -m pip install 'numpy<2' ultralytics
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

For the first test run inference on CPU (safer with Isaac Sim already using GPU memory). The default model is `yolo11n.pt`, which Ultralytics may download on first use: if offline, provide a downloaded weights path using `-p model:=...`.

With a **real** RGB image topic, in the venv:

```bash
python -m asn_semantical_navigation.yolo_detector_node --ros-args -p use_sim_time:=true -p image_topic:=/YOUR_RGB_IMAGE_TOPIC -p device:=cpu
```

Run search-command node in a separate ROS-sourced terminal:

```bash
ros2 run asn_semantical_navigation search_command_node --ros-args -p use_sim_time:=true
```

Select target:

```bash
ros2 topic pub --once /semantic/search/command std_msgs/msg/String "{data: 'find a backpack'}"
ros2 topic echo /semantic/search/target_class
ros2 topic echo /semantic/search/detections_json
```

`yolo_detector_node` publishes confidence, bounding boxes, confirmation count, and `confirmed` status. It will only declare `target_found=true` after the required consecutive frames.

## Test D — combined autonomous search

Restart autonomous node **with defaults** (the text target and recent YOLO frames are required before it can start):

```bash
ros2 run asn_semantical_navigation nbv_auto_node --ros-args -p use_sim_time:=true -p auto_max_goals:=5 -p auto_goal_timeout_s:=90.0
```

Once the target command has been received and YOLO is processing images, use:

```bash
ros2 service call /semantic/nbv/auto_enable std_srvs/srv/SetBool '{data: true}'
```

This is a supervised Isaac Sim validation: while armed, the planner repeatedly sends goals until a verified Nav2 failure/pose mismatch, no useful frontier, an action budget, missing detector frames, or a confirmed YOLO class detection. For debugging:

```bash
ros2 topic echo /semantic/nbv/execution_status
ros2 topic echo /semantic/search/detections_json
ros2 run tf2_ros tf2_echo map base_link
```

## Important limitations, deliberately left for refinement

- **Map frame:** `move(+0.6,+0.0)` is an offset along **map X**, not the robot's forward axis. Nav2 may arc or reverse, but after-goal pose checks must match the map-frame target. If a commanded map-X goal results in body-frame movement and fails the pose check, **autonomy will stop**; do not tune it out without diagnosing TF/odom/control.
- YOLO uses COCO class detection; `find a black backpack` means class `backpack` only. It does not classify black color or verify depth, nor approach objects yet.
- Unknown camera intrinsics: predicted visibility uses a configurable 70° planar approximation. Real camera frames and the existing RGB-D confidence map are the source of truth for measured coverage.
- Frontier fallback approximates reachable free space from a static occupancy map; it may propose a goal Nav2 later rejects due to dynamic obstacles. A rejected/failed goal safely pauses autonomous search.
- Local gain of a translation is predicted **at its endpoint only**, while the real coverage node updates during motion. We will refine full motion/rotation gains later.
- The default 90 s navigation timeout is **wall-clock**, not simulation-time. Prior simulation ran substantially below real time.
- Still pending: actual ROS/Isaac Sim integration validation, calibrated camera model, robust global planning, object localization, safe target approach, repeated-run coverage metrics, speed optimization, LLM reasoning.
