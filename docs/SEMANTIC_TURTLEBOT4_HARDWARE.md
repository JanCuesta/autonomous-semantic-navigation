# TurtleBot 4 Lite hardware sensor pipeline

Robot: `turtlebot05`, `ubuntu@192.168.77.1`. Actual robot topics have no
namespace. Simulation launch files are unchanged. These commands run on the
**laptop**, from `~/projects/autonomous-semantic-navigation/ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="192.168.77.1:11811"
export ROS_SUPER_CLIENT=TRUE
colcon build --packages-select asn_bringup asn_semantical_navigation --symlink-install
source install/setup.bash
ros2 launch asn_bringup hardware_sensors.launch.py
```

Run exactly one sensor launcher. Stop any previous manual
`coverage_depth_resize` / `hardware_depth_cloud` processes first. The nodes
are lazy: output may be absent until a subscriber requests an image/cloud.
The launcher does not start drivers, mapping, localization, navigation, or
any motion controller. All its nodes use `use_sim_time=false`.

In another laptop terminal, source the same environment and workspace:

```bash
ros2 launch asn_bringup semantical_navigation_hardware.launch.py enable_perception:=true
```

This starts target selection, YOLO, object localization and coverage only.
The default without `enable_perception` remains RGB detection only.
The perception Python defaults to the repository `.venv-perception/bin/python`;
it must contain ultralytics and access ROS/cv_bridge and the installed package.
No target or mission must be selected for stationary startup checks.
`enable_full_stack:=true` additionally starts approach, NBV and mission nodes;
use that only after separately verifying map, localization and Nav2. No motion
acceptance was performed during configuration.

## Data paths and calibration

- YOLO consumes `/oakd/rgb/image_raw/compressed` directly as JPEG
  `CompressedImage`, with sensor-data QoS. Its JSON retains the source
  timestamp, optical frame and original image dimensions.
- `hardware_depth_decode` uses Jazzy image_transport **parameters**
  `in_transport=compressedDepth`, `out_transport=raw`, producing full-size
  `/hardware_depth/image_raw` (1280x720, 16UC1). The localizer is an Image
  subscriber, not a compressedDepth decoder. It uses this local raw topic
  and `/oakd/stereo/camera_info`.
- `coverage_depth_resize` consumes compressedDepth plus the stereo CameraInfo,
  with nearest-neighbor decimation 4 in both dimensions. It preserves zero
  invalid depths. Its output is `/coverage_depth/image_raw`, 320x180.
- `/coverage_depth/camera_info` intentionally retains calibration width/height
  1280x720 and K/P, with binning_x/y=4 and full-image ROI. Jazzy image_geometry
  applies binning and ROI when point_cloud_xyz constructs its camera model.
  Effective fx=fy=251.18540955, cx=157.23402405, cy=92.93538666.
  Do not divide K/P again: that would apply decimation twice.
- `hardware_depth_cloud` consumes that reduced image **and its derived
  CameraInfo** using raw transport, producing `/depth_pcl` for coverage.
  Explicit camera remaps are included although image_transport can derive
  the associated CameraInfo names automatically.
- Coverage and localizer use sensor-data QoS. Depth units are 16UC1 mm or
  32FC1 meters; the localizer converts mm to meters, selects valid depths in
  0.15–5 m, requires RGB/depth timestamp separation <=0.10 s, and transforms
  at the measurement timestamp. Coverage further limits ray range.

`robot_namespace` defaults to an empty string; robot map, action, speed-limit
and TF remaps therefore resolve at root. Project `/semantic/*` topics remain
root-level. For a different robot, override namespace and sensor topics
explicitly; the locally generated depth/cloud names stay laptop-local.

## Observed stationary results (2026-10-29)

Installed packages: image_proc/depth_image_proc 5.0.13,
compressed_depth_image_transport 4.0.7, image_transport 5.1.8.
Fresh battery, scan, odom, RGB, depth and calibration messages were received.
The initial working manual converters and the repository replacement both
produced 320x180 clouds with point_step=16, row_step=5120 and 921600 data bytes.
Initial sustained observation: 168 clouds at approximately 5 Hz over 34 s;
RGB/depth nearest timestamp gaps were 1.36–1.72 ms. Original full clouds were
14745600 bytes: the reduced payload is 16x smaller (4.608 MB/s at 5 Hz versus
73.728 MB/s before, excluding DDS overhead). No full-resolution cloud baseline
was rerun; no causal conclusion about Wi-Fi loss is claimed.

Full decoded depth was received at 1280x720, 1843200 bytes, 16UC1.
One stationary sample had approximately 54% nonzero depths, with nonzero
values about 0.53–9.84 m. Far values outside the localizer range are rejected.
Non-dense clouds are expected where depth is zero; a sample had ~53% finite
XYZ points.

TF `odom -> base_link -> oakd_rgb_camera_optical_frame` exists. `map -> odom`
is missing. No map, AMCL/SLAM or Nav2 nodes/action endpoints were discovered.
Map-based target localization and measured coverage therefore cannot complete.
Bring up the appropriate map/localization/Nav2 separately with exactly one
map-to-odom publisher; review stamped velocity configuration separately before
any future motion test.

RGB and depth share frame, resolution and K/P. Stereo CameraInfo has zero
D whereas raw RGB has nonzero rational distortion coefficients. These checks,
and matching timestamps, do **not** establish pixelwise geometric registration.
Spatial RGB/depth edge correspondence and quantitative object position accuracy
remain unverified. Validate with a stationary foreground object against a
background, including image edges, and inspect the RGB distortion/rectification
contract before accepting localized targets. Do not weaken localizer guards.

## Robot configuration

On the **robot**, `/etc/turtlebot4/setup.bash` already sets domain 0,
Fast DDS, empty namespace and `ROS_DISCOVERY_SERVER=127.0.0.1:11811`.
`turtlebot4.service` was active. Its camera YAML already configured RGBD,
5 Hz camera streams and full RGB publication, but lacked a stereo section.
The file was backed up to:

`/opt/ros/jazzy/share/turtlebot4_bringup/config/oakd_lite.yaml.asn-backup-20261029-171135`

Then `stereo.i_align_depth: true` and `stereo.i_output_disparity: false`
were persisted alongside camera/rgb/left/right under ros__parameters.
No service/network restart was performed, so the saved settings have not been
validated across restart. Running camera alignment still requires geometric
verification. The repeated ROS_SUPER_CLIENT lines in setup.bash were left alone.

Interfaces were checked against installed packages and matching upstream sources:
[crop_decimate](https://github.com/ros-perception/image_pipeline/blob/5.0.13/image_proc/src/crop_decimate.cpp),
[point_cloud_xyz](https://github.com/ros-perception/image_pipeline/blob/5.0.13/depth_image_proc/src/point_cloud_xyz.cpp),
[republish](https://github.com/ros-perception/image_common/blob/5.1.8/image_transport/src/republish.cpp).

Final replacement-launch check, after stopping surviving old converter children:
exactly one reduced-image and one cloud publisher. Received 166 clouds at
4.97 Hz and 155 full decoded depth images at 4.64 Hz over ~34 s. Thus full-depth
subscriber delivery did not sustain all source frames in this check; the cause
was not isolated. Nearest cloud/RGB stamp gaps remained 1.36–1.72 ms. The
all-source-image comparison includes ~0.20 s maxima at capture boundaries;
it is not proof of a synchronized-pair failure. Sampled cloud XYZ agreed with
the binning-adjusted projection within 0.9 micrometers.

Both affected packages built successfully. Thirteen existing unit tests passed;
broader collection fails because test_nbv_core imports missing
safe_straight_segment. Perception-only startup reached YOLO-ready and started
localizer/coverage without motion nodes. The timed shutdown produced existing
KeyboardInterrupt traces. Pixelwise registration remains unverified.

## Consistent terminals and stationary RViz

Source `~/projects/autonomous-semantic-navigation/ros2_ws/hardware_env.bash`
in **every** laptop terminal before running launch, topic pub, or topic echo.
This sources ROS and the workspace and sets the same DDS/discovery environment.
A terminal without ROS_DISCOVERY_SERVER may wait indefinitely for subscribers
although the hardware nodes are running.

The hardware semantic launcher now starts RViz by default; disable it with
`use_rviz:=false`. Its stationary profile uses `odom` as fixed frame and omits
navigation-goal tools. It shows scan, odometry, reduced cloud and detector image.
The annotated detector image requires a selected target. Map and coverage
visualization require map localization and are excluded from this sensor profile.
For an already running semantic launch, open RViz separately:

```bash
source ~/projects/autonomous-semantic-navigation/ros2_ws/hardware_env.bash
ros2 run rviz2 rviz2 -d ~/projects/autonomous-semantic-navigation/ros2_ws/src/asn_bringup/config/hardware_stationary.rviz
```

## Supervised semantic startup (November 4)

Run on the laptop:

```bash
cd ~/projects/autonomous-semantic-navigation
./turtlebot4_semantic.sh full
```

This sources the hardware environment, checks scan/odom timestamps, and starts
one sensor pipeline, saved-map Nav2/localization, and full semantic stack.
The default map is ros2_ws/maps/my_environment.yaml; optionally supply a map
as the second argument. Set the actual 2D Pose Estimate in the map-based RViz
view. Autonomy starts disarmed and mission state IDLE; this command does not
select a target, start a mission, or send a goal. Ctrl+C stops all supervised
launches. It refuses to start if project nodes are already running. Stop their
original launch terminals first. Camera-only checking with localization omitted:
`./turtlebot4_semantic.sh perception` (requires live scan/odom preflight).

The full-mode RViz view includes Nav2 pose controls plus semantic coverage,
target markers, reduced cloud and annotated detection image. Software rendering
is enabled in the hardware semantic launch to avoid the observed indexed-map
GLSL error; override software_rendering:=false for hardware rendering.

Launch booleans enable_full_stack and enable_perception now consistently accept
true/1/yes. The full-mode argument is declared before RViz evaluates it, so
running with default arguments also resolves correctly. Hardware component
container names now include the launch process ID: this avoids reusing the same
service name across rapid restarts, which stalled component loading in testing.

November 4 startup observations: robot reported 99% battery and synchronized
clocks. Full semantic nodes started; mission IDLE and NBV AUTO_DISARMED were
confirmed. Coverage consumed the calibrated reduced cloud once operator pose
was set. There were transient lifecycle heartbeat resets and intermittent sensor
delivery during initial startup; bond/sensor safety timeouts were not disabled
or enlarged. These observations do not establish reliable autonomous motion.
Spatial RGB/depth registration and target position accuracy remain unverified.
