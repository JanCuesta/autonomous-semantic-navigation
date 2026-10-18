# System architecture

The ROS 2 workspace integrates a differential-drive simulated robot, Isaac Sim sensing, `ros2_control`, a wheel-derived odometry node, AMCL/SLAM Toolbox, Nav2 and a custom C++ mission manager. This diagram reflects the checked-in code and config, **not** an independent end-to-end run certification.

```mermaid
flowchart TD
    SIM[Isaac Sim robot / environment] --> BRIDGE[ROS 2 bridge and ros2_control]
    BRIDGE --> SCAN[/scan, /clock, /joint_states]
    SCAN --> WODOM[Wheel-derived /odom + odom → base_link]
    SCAN --> LOC[AMCL saved-map localization or SLAM Toolbox mapping]
    WODOM --> LOC
    LOC --> NAV[Nav2 planner / controller / costmaps]
    GOALS[RViz publishes /mission/goal] --> MM[C++ mission manager]
    MM -->|/navigate_to_pose action| NAV
    NAV --> SMOOTH[Velocity smoothing / collision monitor]
    SMOOTH --> CTRL[diff_drive_controller]
    CTRL --> BRIDGE
    MM --> STATUS[/mission/state]
```

## Package-level responsibilities

| Package | Role |
|---|---|
| `asn_bringup` | `simulation.launch.py`, `navigation.launch.py`, `localization.launch.py`, `mapping.launch.py`, hardware launch profiles |
| `asn_description` | ROS robot URDF and robot-state publishing resources |
| `asn_simulation` | Diff-drive controller configuration and `asn_wheel_odom_unwrapped` executable |
| `asn_navigation` | Nav2, AMCL and SLAM parameters, occupancy maps and RViz presets |
| `asn_mission` | C++ `mission_manager` ROS node |
| `asn_semantical_navigation` | Experimental Python depth-coverage node, **not** semantic object recognition |
| `asn_test` | ROS package test scaffold; additional scripts under root `tests/` |

## Mission manager interfaces

| Topic / service / action | Type | Use |
|---|---|---|
| `/mission/goal` | `geometry_msgs/msg/PoseStamped` | Receive mission goal |
| `/mission/state` | `std_msgs/msg/String` | Report mission state |
| `/mission/cancel` | `std_srvs/srv/Trigger` | Request active cancellation |
| `/navigate_to_pose` | Nav2 action | Goal submission and feedback/result |
| `/controller_manager/list_controllers` | ROS service | Confirm active diff-drive controller |
| `/odom` | `nav_msgs/msg/Odometry` | Check wheel odometry freshness |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Check availability of localization estimate |

The code tracks `IDLE`, `PREPARING`, `NAVIGATING`, `SUCCEEDED`, `FAILED`, `CANCELLED`. Defaults: **180 wall-second** goal timeout (per attempt), **3** maximum attempts, **1 wall-second** odometry freshness. Code implementation is not equivalent to completed fault-injection validation.

## Timing and coordinate frames

For saved-map localization, **AMCL** should own `map → odom`. During mapping, **SLAM Toolbox** owns it. The custom wheel odometry node publishes `odom → base_link` and `/odom`; `robot_state_publisher` supplies `base_link → laser_frame`. Running mapping and localization simultaneously without managing TF ownership can create conflicts.

```
map → odom → base_link → laser_frame
```

Controller wheel separation is **0.233 m**; effective *odometry-model* separation is **0.23822 m**, with wheel radius **0.03575 m**. The current localization launcher references **`v4_sim_map.yaml`**. The included RViz `v3_rviz.rviz` uses `/mission/goal` for its goal tool; its path visualization is configured but disabled by default.

**Third-party attribution:** Nav2 supplies planning/control plugins; AMCL supplies saved-map localization; SLAM Toolbox supplies mapping; ros2_control supplies controller infrastructure. This repo integrates/configures them and adds project-specific nodes and tests.
