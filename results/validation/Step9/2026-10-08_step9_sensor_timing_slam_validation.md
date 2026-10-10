# Step 9 — Sensor, Timing, TF and SLAM Integration Validation

**Project:** Autonomous Semantic Navigation Robot — V1 simulation  
**Date:** 2026-10-08  
**Environment:** Ubuntu 24.04 · ROS 2 Jazzy · Isaac Sim 6.1.0-rc.26 · Create 3 simulation  
**Status:** **FUNCTIONAL INTEGRATION VALIDATED** — real-time performance acceptance remains open.

## Outcome

Validated the live ROS 2 sensor, time and transform pipeline after the Step 8B caster and wheel-odometry calibration. A single passive recorder was used for stationary, commanded-motion and SLAM-load runs. The robot maintained 10 Hz simulated LiDAR and approximately 50 Hz simulated joint states and odometry across all three runs, without recorder warnings or errors. SLAM Toolbox reached its active lifecycle state, published `/map`, and supplied the `map -> odom` transform. The validated interfaces support moving on to **Step 10: saved-map and AMCL localization validation**.

## Work completed

- Confirmed `diff_drive_controller` and `joint_state_broadcaster` were active; retained calibrated wheel radius **0.03575 m** and odometry-effective wheel separation **0.23822 m** (controller nominal separation **0.233 m**).
- Ran passive `asn_sensor_timing_recorder.py` in three phases: **stationary** (60 simulated seconds), **motion** (40 simulated seconds) and **slam_load** (60 simulated seconds).
- Checked ROS topics `/clock`, `/scan`, `/joint_states`, `/odom`, `/tf`, and `/tf_static` for simulated-time rates, monotonicity, TF ownership and continuity.
- Ran an independent short command sequence during the motion recording: forward travel, in-place left rotation, settles and idle, with zero velocity commanded at completion.
- Launched SLAM Toolbox from `mapping.launch.py` under sensor load, verified lifecycle **active [3]**, a live occupancy grid on `/map`, and `map -> odom` publication; no persistent RViz TF errors were observed after startup.
- Verified the navigation odometry chain `map -> odom -> base_link -> laser_frame` (mapping mode), with `odom -> base_link` owned by `asn_wheel_odom_unwrapped` and `base_link -> laser_frame` provided by the robot description.

## Key results

| Measurement (simulated-time unless noted) | Stationary | Motion | SLAM load |
| --- | ---: | ---: | ---: |
| Mean real-time factor (sim/wall) | 0.4584 | 0.4110 | 0.4128 |
| `/scan` rate | 10.00 Hz | 10.00 Hz | 10.00 Hz |
| `/joint_states` rate | 50.00 Hz | 50.00 Hz | 49.99 Hz |
| `/odom` rate | 49.99 Hz | 50.00 Hz | 50.00 Hz |
| Recorder warnings / errors | 0 / 0 | 0 / 0 | 0 / 0 |

- LiDAR: **3,600 beams**, `laser_frame`, published range limits **0.10–200 m**; scan stamps advanced monotonically during the detailed stationary/motion reviews.
- Timing: no backwards `/clock` events in the reviewed runs. Stationary 5-second wall-window RTF minimum **0.395**; motion minimum **0.397**. These brief dips are tracked as performance observations rather than functional sensor failures.
- Stationary odometry start-to-end drift: **6.98 mm** without SLAM and **7.07 mm** under SLAM load. These measurements do not distinguish physical settling from odometry integration drift.
- Commanded motion regression: **0.1414 m** displacement during a nominal **0.15 m** short forward command; approximately **40.35°** left turn during the turn interval plus **1.95°** settling rotation. This was an interface regression, **not** an independent Isaac-ground-truth odometry-accuracy test.
- Under SLAM load, observed dynamic TF pairs included `map -> odom` (**7,194 samples**) and `odom -> base_link` (**2,999 samples**); static `base_link -> laser_frame` was also observed. `/map` had **one publisher** during the SLAM test.
- ROS simulated time and sensor rates stayed consistent while mean RTF was well below 1.0: 10 simulated Hz is not 10 wall-clock Hz when RTF is approximately 0.41.

## Decision and limits

**PASS:** functional LiDAR, joint-state, odometry, simulated-clock, TF and SLAM integration checks for the V1 simulated robot. No change to sensor range, calibrated geometry or Isaac Safe launch configuration was justified by these runs.

**OPEN:** the original provisional real-time-performance requirement (mean RTF >= 0.95; no prolonged RTF < 0.90) was **not met**, and must not be silently relaxed. Detailed end-to-end command latency, fault injection, real-time deployment readiness, and quantitative map/localization accuracy were not established here. The passive recorder's zero warnings/errors are evidence for its implemented checks, not a guarantee that every TF lookup will succeed.

## Step 10 handoff — New reference map (not yet localization-validated)

After Step 9, a new SLAM mapping run was performed with the calibrated configuration, starting from a **known Isaac world pose**: `base_link` approximately **(X = 0.5000 m, Y = 0.5002 m, yaw = 89.99°)**. The robot was driven through the two rooms and corridor; a new occupancy map was saved as:

- `ros2_ws/src/asn_navigation/maps/v2_sim_map.yaml`
- `ros2_ws/src/asn_navigation/maps/v2_sim_map.pgm`

V2 metadata: **0.05 m/cell**, **161 x 101 cells**, map-grid origin **[-0.522, -4.532, 0]**. The original `v1_sim_map.*` files were preserved. The saved map visually includes the rooms, corridor and obstacles; some incomplete/uneven boundary observations were noted and accepted as a **known limitation for testing**, not proof of geometric accuracy. The Isaac world start pose is a known reference but **does not itself define** the final ROS `world -> map` transform.

**Next (Step 10):** stop SLAM; load V2 in `map_server`; initialize AMCL; check unique `/map` and `map -> odom` ownership; verify scan/map correspondence; establish independent world-to-map alignment and measure localization position/heading error and repeatability. Nav2 goal and fault testing follows localization validation.

## Repository / evidence

- Recorder: `tests/validation/Step9/asn_sensor_timing_recorder.py`
- Motion driver: `tests/validation/Step9/asn_step9_motion_driver.py`
- Stationary: `results/validation/Step9/stationary_20261008T224033Z/`
- Motion: `results/validation/Step9/motion_20261008T224907Z/`
- SLAM load: `results/validation/Step9/slam_load_20261008T225544Z/`
- Mapping launch/config: `ros2_ws/src/asn_bringup/launch/mapping.launch.py`; `ros2_ws/src/asn_navigation/config/slam_toolbox_mapping.yaml`
- Suggested report destination: `results/validation/Step9/2026-10-08_step9_sensor_timing_slam_validation.md`

This report summarizes the evidence shared during testing; no workstation files or repository commits were modified by generating it.
