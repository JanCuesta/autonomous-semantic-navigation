# Documentation index

## Recruiter- and engineer-facing guides

| Guide | Content |
|---|---|
| [Architecture](ARCHITECTURE.md) | ROS 2 packages, interfaces, TF and mission manager |
| [Getting started](GETTING_STARTED.md) | Dependencies, build and run instructions |
| [Validation](VALIDATION.md) | What has been measured; limitations and raw-data links |
| [Project status](PROJECT_STATUS.md) | Completed/in progress/planned scope |
| [Media checklist](../media/README.md) | Exact filenames and suggested demonstration footage |

## Original engineering records

- [Source-of-Truth v1.1 (PDF)](Autonomous_Semantic_Navigation_Robot_Source_of_Truth_v1.1.pdf) — planning/specification document, not proof that all proposed features are implemented.
- [Provisional V1 specifications](V1%20specs%20%28provisional%29/) — concept of operations, design, traceability, risk and sim-to-hardware baseline.
- [Daily development tracker](debugging/DAILY_TRACKER.md) · [debugging log](debugging/debugging_log.csv) · [controller-manager recovery note](debugging/2026-09-30_ros2_control_controller_manager_recovery.md).

## Validation evidence

- [Step 8B — odometry/caster calibration report](../results/validation/Step8/final_report_odometry_validation_2026-10-08.md)
- [Step 9 — sensor timing, TF and SLAM report](../results/validation/Step9/2026-10-08_step9_sensor_timing_slam_validation.md)
- [Step 7 — initial simulation feedback report](../results/step7/2026-10-01_step7_simulation_feedback_report.md)
- [Raw results](../results/) · [Validation scripts](../tests/validation/)

Original reports can describe dated setups or maps that have since changed. The checked-in launch files govern **current configuration**, and the dated reports govern **historical measured claims**.
