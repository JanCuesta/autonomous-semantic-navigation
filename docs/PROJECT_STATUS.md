# Project status and roadmap

**October 2026 documentation snapshot**, grounded in repository code and the existing validation reports.

## Implemented in repository

- Isaac Sim robot/world assets and ROS 2 bridge-oriented integration.
- Controller-manager spawner launch, diff-drive controller configuration, robot-state publisher, and custom wheel odometry.
- SLAM Toolbox mapping, AMCL saved-map localization, Nav2 configuration and multiple occupancy maps.
- C++ `mission_manager` node with goal/action, readiness, state, cancellation, timeout and retry logic.
- Experimental Python depth-coverage node (`asn_semantical_navigation`): **not** object-level recognition or full semantic navigation.
- Test harness scripts and recorded wheel-odometry/sensor/SLAM reports.

## Supported by measured evidence

- Selected motion/odometry maneuvers under a calibrated, idealized caster model (Step 8B).
- Functional clock/sensor/TF/SLAM integration with a documented sub-real-time simulator (Step 9).

## Open work

1. Ground-truth saved-map AMCL localization error and repeatability with the **current V4 map**.
2. Repeated autonomous navigation missions with recorded goal completion, trajectory and time.
3. Mission manager fault/cancel/timeout/retry tests and safety-stop validation.
4. Quantified simulation-time versus wall-clock performance and its effect on safety/control.
5. Physical robot deployment and sim-to-real calibration (V2).
6. Object recognition/depth fusion and semantically driven goal selection (V3).

## Design stages

| Stage | Status | Interpretation |
|---|---|---|
| V1 simulation | **In progress** | Sensors/odometry/mapping validated in selected tests; navigation reliability and safety still open |
| V2 physical robot | **Planned** | Hardware launch profiles/check scripts do not imply deployment |
| V3 semantic autonomy | **Experimental/planned** | Depth-coverage prototype is not a complete semantic perception stack |

## Workstation and version differences

Canonical development world for the README: `isaac_sim/worlds/Testing_World+Robot.usd`. The repository preserves `Validation.usd`, `Validation-2.usd`, and `indoor_navigation_v1.usd` as alternative/historical scenes. `localization.launch.py` currently selects `v4_sim_map.yaml`; older reports may refer to V2. **Neither the file name nor its version implies a validated localization accuracy.**

This remains a technical development/validation project, not certified robotics software. Credit third-party Nav2/AMCL/SLAM/ros2_control functionality accurately.
