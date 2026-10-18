# Getting started — V1 Isaac Sim / ROS 2

> Derived from checked-in launch files and dated development reports. **Not clean-install tested during this documentation update.** The local Isaac Safe launcher used in development is not distributed in this repository.

## Environment and dependencies

- Ubuntu **24.04**, ROS 2 **Jazzy**, NVIDIA Isaac Sim **6.1.0-rc.26** (reported in October 2026 tests).
- Working Isaac Sim ROS bridge and `controller_manager` setup, Nav2, SLAM Toolbox, AMCL, ros2_control, RViz2, colcon and rosdep.
- Hardware configuration differs from simulation; this guide documents **simulation mode only**.

## Build

```bash
git clone https://github.com/JanCuesta/autonomous-semantic-navigation.git
cd autonomous-semantic-navigation/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`rosdep` resolves declared ROS dependencies but does not install/configure Isaac Sim and may not capture all Python dependencies in exploratory scripts. On every additional terminal, source `/opt/ros/jazzy/setup.bash` and the workspace `ros2_ws/install/setup.bash`.

## Run (separate terminals)

1. Start the already-configured Isaac Sim instance with its working bridge/controller manager. Open `isaac_sim/worlds/Testing_World+Robot.usd` and press **Play**. There are older/alternative world files; do not assume they are interchangeable.
2. Start the ROS 2 simulation-side controller spawners, wheel odometry and robot state publisher:

```bash
ros2 launch asn_bringup simulation.launch.py
```

3. Start the saved-map AMCL localization and Nav2 bringup:

```bash
ros2 launch asn_bringup navigation.launch.py
```

4. Start the **C++** mission-manager executable:

```bash
ros2 run asn_mission mission_manager
```

5. Start RViz using the included preset:

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix asn_navigation)/share/asn_navigation/rviz/v3_rviz.rviz"
```

Current localization launch configuration selects **`v4_sim_map.yaml`**. In RViz, use **2D Pose Estimate** to initialize AMCL, then the configured **2D Goal Pose** tool publishes to `/mission/goal` (not directly to Nav2). Wait for odometry, localization, controller and Nav2 readiness. Starting the processes is **not** itself evidence of a successful navigation mission.

### Inspection and debugging

```bash
ros2 control list_controllers
ros2 topic echo /mission/state
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /amcl_pose --once
ros2 action list
```

For an active mission only, `ros2 service call /mission/cancel std_srvs/srv/Trigger '{}'` requests cancellation; actual termination is established by the Nav2 action result. The RViz Path display is initially disabled and configured for `/transformed_global_plan`; check the currently published path topic before recording it as a demonstration.

## Mapping mode — use instead of saved-map AMCL mode

With Isaac Sim and the simulation ROS interfaces running:

```bash
ros2 launch asn_bringup mapping.launch.py
```

This launches SLAM Toolbox and the mapping RViz config. Do **not** run mapping and AMCL localization with duplicate `/map` and `map → odom` ownership unless explicitly configured to avoid conflicts.

## Troubleshooting

| Symptom | Likely checks |
|---|---|
| Spawner fails or waits | Isaac Sim playing; `/controller_manager` accessible; controller already active? |
| No `/scan` or `/clock` | Sim bridge and expected world/robot scene loaded |
| Missing `/odom` | Wheel joint feedback and wheel-odom executable; inspect simulation time |
| RViz TF errors | One owner per transform; correct `map`, `odom`, `base_link`, `laser_frame` chain |
| AMCL pose unavailable | Initialization, saved V4 map alignment and scan compatibility |
| Mission manager `NOT READY` | `/odom`, `/amcl_pose`, Nav2 action and diff-drive controller active |
| Low apparent sensor rate | Inspect simulated time vs wall time: October mean RTF ~0.41–0.46 |

More detail: [architecture](ARCHITECTURE.md), [validation](VALIDATION.md), [controller recovery record](debugging/2026-09-30_ros2_control_controller_manager_recovery.md).
