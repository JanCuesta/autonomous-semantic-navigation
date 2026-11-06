# TurtleBot 4 Lite mapping and navigation (laptop)

All commands run on the laptop. The robot bringup must already be active.
The wrapper sources ROS Jazzy, the workspace, domain 0, Fast DDS, and discovery
server 192.168.77.1:11811. It does not send navigation goals automatically.

Build once after pulling/editing:

```bash
cd ~/projects/autonomous-semantic-navigation/ros2_ws
source hardware_env.bash
colcon build --packages-select asn_navigation asn_bringup --symlink-install
```

## Map the environment

```bash
cd ~/projects/autonomous-semantic-navigation
./turtlebot4_navigation.sh mapping
```

This starts SLAM Toolbox and mapping RViz using `/scan`, `odom`, `base_link`,
real time, and the hardware SLAM YAML. SLAM publishes `/map` and `map -> odom`.
Do not run AMCL/localization or a second SLAM instance at the same time.
Map exploration requires separately supervised manual operation of the robot;
no motion or teleoperation was tested during this configuration task.

In another laptop terminal, save the map while SLAM is still running:

```bash
cd ~/projects/autonomous-semantic-navigation
./turtlebot4_navigation.sh save-map "$PWD/ros2_ws/maps/my_environment"
```

This writes my_environment.yaml and my_environment.pgm. Keep both together.
Then Ctrl+C the mapping launch before starting saved-map navigation.

## Localize and send location goals later

```bash
cd ~/projects/autonomous-semantic-navigation
./turtlebot4_navigation.sh navigation "$PWD/ros2_ws/maps/my_environment.yaml"
```

RViz opens the installed Nav2 panel/profile. Set **2D Pose Estimate** to the
robot's actual position and heading on the saved map. Wait for AMCL to match
the scan/map and for Nav2 lifecycle activation. Confirm pose and obstacle
placement before using the Nav2 RViz goal tool to choose a destination.
A navigation goal can move the robot. No goals were sent during these checks.
Do not run the semantic mission controller concurrently with manual Nav2 goals.

Localization alone is available with the wrapper's `localization MAP.yaml` mode.
It deliberately does not start navigation or RViz. All navigation output is
TwistStamped on `/cmd_vel`. Controller and behavior commands pass through
`cmd_vel_nav`, the velocity smoother, and collision monitor using the installed
Nav2 launch routing. Docking components come from installed Nav2 bringup and
have no configured dock database; do not use docking actions through this profile.

Normal launch names support hardware explicitly:

```bash
source ~/projects/autonomous-semantic-navigation/ros2_ws/hardware_env.bash
ros2 launch asn_bringup mapping.launch.py hardware:=true
ros2 launch asn_bringup navigation.launch.py hardware:=true map:=/absolute/path/my_environment.yaml
```

Without hardware:=true, the original simulation launch behavior is retained.
Direct hardware launch names mapping_hardware.launch.py,
localization_hardware.launch.py, navigation_hardware.launch.py remain supported.
Optional `use_rviz:=false` disables RViz. Navigation/localization require a real
map argument; they no longer silently choose a missing lab or simulation map.
Source hardware_env.bash in every terminal running raw ROS CLI commands.

## Validation and limits

Stationary SLAM activated, consumed the real lidar scan and saved a 242x256 map
at 0.05 m/cell. That temporary test map covers only the stationary view and is
not a completed map of the environment.

Separate-process Nav2 encountered long discovery/lifecycle-service delays.
Hardware navigation therefore uses a shared component container, including
container-level YAML for child costmaps. Localization activated with the test
map. Stamped velocity parameters are enabled on controller, behaviors,
velocity smoother, collision monitor and docking, as required by the robot's
verified /cmd_vel interface. Hardware base frames use base_link; scan is /scan.
The hardware SLAM maximum range is 12 m, matching its observed lidar capability.

Nav2 testing redirects velocity output with
`cmd_vel_topic:=/asn_validation/cmd_vel`. This is a stationary validation override,
and it overrides intermediate command remaps as well. Use the default /cmd_vel
for normal operation to retain the installed Nav2 command chain. No velocity
messages or navigation goals were sent by the validation tools.

Motion tracking, collision stopping, footprint fit, speed/acceleration tuning,
full environment mapping and real destination arrival remain unverified. Existing
hardware controller speed limits were retained and need review before motion.

Reference: [Nav2 Jazzy stamped velocity configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/).

Final stationary startup result: both localization and navigation lifecycle
managers reported all managed nodes active after AMCL received an initial pose
at the temporary map origin. No destination goal was issued. Rapid restart of
the generic nav2_container also stalled component loading once; the hardware
profiles now consistently use the project-specific asn_nav2_container name.

Files changed for this mapping/navigation task:

- `turtlebot4_navigation.sh` (new wrapper)
- `turtlebot4_hardware_check.sh` (DDS environment and lidar frame)
- `ros2_ws/src/asn_bringup/launch/{mapping,localization,navigation}.launch.py`
- `ros2_ws/src/asn_bringup/launch/hardware/{mapping,localization,navigation}_hardware.launch.py`
- `ros2_ws/src/asn_bringup/package.xml` (runtime dependencies)
- `ros2_ws/src/asn_navigation/config/hardware/slam_toolbox_mapping.yaml`
- `ros2_ws/src/asn_navigation/config/hardware/amcl_localization_hardware.yaml`
- `ros2_ws/src/asn_navigation/config/hardware/nav2_navigation.yaml`
- `ros2_ws/src/asn_navigation/rviz/hardware/mapping.rviz`
- `docs/TURTLEBOT4_MAPPING_NAVIGATION.md` (this guide)

Earlier semantic-pipeline edits were preserved. No robot configuration was
changed during this mapping/navigation task. Package builds, launch Python
parsing, shell syntax and git diff whitespace checks passed.

## Robot clock after reboot

The robot was observed booted with a March 13 clock while the laptop was on
October 29. Collision Monitor rejected scans ~19.9 million seconds old and
blocked velocity output, despite Nav2 issuing commands. Do not disable scan
freshness checks. The robot AP has no internet route, so its configured public
NTP pools cannot synchronize it.

With mapping, navigation and teleop stopped, run on the laptop:

```bash
cd ~/projects/autonomous-semantic-navigation
./turtlebot4_sync_time.sh
```

This authenticates SSH locally, sets Raspberry Pi and Create 3 base time from the laptop, and restarts
only turtlebot4.service to reset sensor timestamp offsets. Networking is not
restarted. Wait for bringup, then launch navigation again and set the actual
initial pose. Repeat after robot reboot if its clock is wrong. The navigation
wrapper now refuses to start if either /scan or /odom timestamp differs by more than 0.5 s,
or if no scan arrives within 45 s. The helper sets time for the session; it
does not provide continuous NTP synchronization.

The Create 3 base clock is separate: correcting Pi time alone fixed lidar but
left odometry/TF old. Its installed Iron firmware uses the POST form
/beta-datetime-save with newdatetime in UTC (verified from /beta-datetime);
the helper updates that as well. Direct base date update returned HTTP 200.
The robot chrony server already allows the USB subnet and serves local time;
no NTP configuration changes were needed. Old goals were discarded by stopping
Nav2 before changing clocks. Restart navigation and reset actual initial pose.

After a robot reboot, run clock recovery again before navigation. A subsequent
check confirmed that both Pi and base returned to March 13 after reboot; the
one-session clock correction does not survive that boot behavior. The helper
now stops bringup before changing time, computes the timestamp after shutdown,
sets both clocks, starts bringup, and checks /scan and /odom before success.
Keep navigation and teleop stopped during recovery. No reboot or networking
restart is needed for this recovery.

Clock recovery now verifies the Pi clock against three SSH midpoint measurements
(using the lowest RTT) and compensates transport delay before setting time.
The final clock offset must be below 0.25 s. Both sensor streams must then stay
within 0.5 s of laptop time for three continuous seconds. This replaces the old
single-message, two-second check which admitted a one-second clock offset.
The base is synchronized through its verified time form and NTP resync endpoint.

### Connection startup and camera verification

After connecting the laptop to the robot network, stop navigation, mapping and
teleop and run `./turtlebot4_sync_time.sh` on the laptop. Check that the
laptop's date and time are correct first: its clock is the reference. The script
requires SSH access and passwordless sudo on the robot. It temporarily stops and
restarts robot bringup, sets the Pi and Create 3 clocks, and does not restart
networking. Run it again after robot reboot or if clocks drift; it is not a
continuous synchronization service. Do not change the clock while the robot is
moving or Nav2 is active.

The final laptop-side check requires `/scan` and `/odom` timestamps and receipt
within 0.5 seconds, plus RGB, compressed depth and stereo camera calibration
within 2 seconds, continuously for 3 seconds. Camera timestamps are checked as
published; the script does not directly set an independent OAK-D or lidar clock.
A missing, old, or future message fails the check and identifies its topic.
Camera checks depend on the robot publishing the configured compressed streams.
The mapping/navigation launch preflight still requires scan and odometry only.
The camera's 2-second threshold tests stream availability and gross clock offset,
not precise RGB-depth synchronization or a guarantee of perception performance.

If bringup stops the sensors while docked, the startup script requests the lidar
and camera services to start and repeats the timestamp check. These requests do
not command the drive wheels. The final check must pass before treating the
robot as ready; a successful service call alone does not prove data delivery.
