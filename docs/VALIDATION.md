# Recorded validation evidence

**No new experiments were run for this portfolio update.** All claims below are extracted from the project's existing dated reports and should be interpreted with their listed limitations.

## Step 8B — caster/odometry calibration

[Full Step 8B report](../results/validation/Step8/final_report_odometry_validation_2026-10-08.md) · [raw experiment archive](../results/validation/Step8/)

The evaluation compared independent Isaac articulation-root ground-truth pose against ROS wheel-derived odometry. Ground-truth pose was **not** used to publish navigation odometry.

| Metric | Result |
|---|---:|
| Controller wheel separation | 0.233 m |
| Effective wheel separation in odometry model | 0.23822 m |
| Wheel radius | 0.03575 m |
| Worst stationary endpoint yaw error across new six-turn set | **3.40°** |
| Reopened-world forward-left arc endpoint position/yaw error | **1.63 cm / +2.82°** |
| Reopened-world reverse-right arc endpoint position/yaw error | **1.27 cm / +1.17°** |
| Reopened-world stopping times | 0.358 and 0.425 **simulated** seconds |

Zero-friction caster was an **idealized simulation contact model**, not a measured physical caster property. The effective odometry separation is a calibration parameter, not a physical axle measurement. Curved motion varied between runs, and these trials do not establish navigation success rate.

## Step 9 — sensors, clock, TF and mapping

[Full Step 9 report](../results/validation/Step9/2026-10-08_step9_sensor_timing_slam_validation.md) · [recordings](../results/validation/Step9/)

| Metric (unless otherwise noted, per simulated time) | Stationary | Motion | SLAM load |
|---|---:|---:|---:|
| LiDAR `/scan` | 10.00 Hz | 10.00 Hz | 10.00 Hz |
| `/joint_states` | 50.00 Hz | 50.00 Hz | 49.99 Hz |
| `/odom` | 49.99 Hz | 50.00 Hz | 50.00 Hz |
| Mean real-time factor (sim/wall) | **0.4584** | **0.4110** | **0.4128** |
| Recorder warnings/errors | 0/0 | 0/0 | 0/0 |

Additional checks: 3,600 LiDAR beams with reported 0.10–200 m limits, monotonically advancing simulated clock in reviewed runs, observed `odom → base_link` and `map → odom` TF, live `/map`, and active SLAM Toolbox.

**Important failure/limitation:** provisional real-time acceptance target (mean RTF **≥ 0.95**) was **not met**. A 10 Hz simulated-time scan is not 10 Hz in wall-clock time when the simulator runs at ~0.41× real time.

## Acceptance status (do not overclaim)

| Acceptance question | Evidence status |
|---|---|
| Wheel-derived odometry and controlled turning/arc calibration | Recorded experimental evidence; limitations above |
| Live LiDAR, odometry, simulated clock, TF integration | Functionally tested in Step 9 |
| SLAM Toolbox active; `/map` and mapping-mode TF | Functionally tested in Step 9 |
| Saved-map AMCL localization ground-truth error/repeatability on current map | **Not established by these reports** |
| Multiple independent autonomous start-to-goal missions | **Not established by these reports** |
| Mission manager timeout, cancellation, retry and fault response | Code exists; **full acceptance remains open** |
| Collision prevention and formal safety requirements | **Not established by these reports** |
| Real-time performance | **Fail relative to provisional target** |
| Physical deployment or object-level semantic navigation | Future work |

The October Step 9 report references a newly saved **V2** map; the current checked-in localization launcher references **V4**. Later map files do not retroactively establish quantitative localization results.

## Next measurements to capture

For each navigation demo: scene and map version, initial pose, requested goal, actual trajectory/planned path, success or failure, time in simulated and wall units, independent endpoint error, collisions/near misses, and any Nav2 recoveries. Repeat runs before claiming reliability or success rate.
