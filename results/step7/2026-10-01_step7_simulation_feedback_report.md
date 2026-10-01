# Step 7 — Simulation timing and wheel feedback

Session: September 30–October 1, 2026. Ubuntu 24.04 / ROS 2 Jazzy; Isaac Sim 6.1.0-rc.26, Kit 110.3.0; `Testing_World+Robot.usd`, `/World/V1_Robot`.

## Outcome and decision

One controlled translation demonstrated close agreement between actual travel and position-based odometry. Raw wheel velocity remains inconsistent with angle change. Stop troubleshooting this issue for now and proceed with TF/frame and SLAM integration. **Full Step 7 acceptance remains open.**

Keep the Isaac Safe launcher, `position_feedback: true`, `open_loop: false`, and original physics settings. CPU boost was restored to `0`. No repository commit was made during this session.

## Timing, LiDAR and controller startup

- Observed real-time factor (RTF) was generally 0.43–0.59, with one run at 0.632. `/clock` remained 60 Hz and `/scan` 10 Hz **per simulated second**. About 5–6 scans per wall second follows from slow simulation; doubling sensor frequency would not restore real time.
- The paired boost test improved RTF from 0.525 to 0.559 (~6.5%); CPU peaks were 55.9°C and 57.9°C. Temperatures around 60–62°C and isolated GPU samples did not establish the performance bottleneck. No further tuning was adopted.
- LiDAR near range was corrected from 1.0 to 0.1 m and saved after backup. `/scan` used `laser_frame`, 3600 beams and 0.10–200 m limits; about 1020 returns below 1 m confirmed close returns were available.
- After a world restart, controller_manager existed with no loaded controllers and unclaimed joint interfaces. Loading and activating `joint_state_broadcaster` and `diff_drive_controller` from the real project YAML restored feedback and odometry. Both used simulation time. Automatic activation remains pending.

Retained configuration: `ros2_ws/src/asn_simulation/config/ros2_controllers.yaml`; manager 50 Hz, wheel radius 0.03575 m, separation 0.233 m, odometry 30 Hz, command timeout 0.5 s. Input: `/diff_drive_controller/cmd_vel` (`TwistStamped`); output recorded: `/diff_drive_controller/odom`.

## Controlled motion result

`asn_controlled_motion.py` recorded raw Isaac states and ROS feedback during 1 simulated second at zero, 1 at 0.05 m/s, then 2 at zero. The expected steady wheel target was 1.398601 rad/s. A recorder lifetime bug initially caused an early exit; retaining its simulation-view owner allowed the final recording to complete.

| Measurement | Result |
|---|---:|
| Command integral | 50.00 mm |
| Isaac root displacement | 46.68 mm |
| Position-based ROS odometry | 45.96 mm |
| Wheel-angle distance | 46.00 mm |
| Odometry versus actual | −0.72 mm (−1.54%) |
| Drift in final stopped second | 0.104 mm |

Acceleration/braking limits can affect the ideal command distance. World and odometry axes differ; heading was not recorded. This validates distance agreement in one short trial, not full pose, turning or repeatability.

## Deferred issue

During the move, raw Isaac left angle change/integrated velocity were 1.118693/1.119503 rad; right values were 1.207123/2.769115 rad. During the final stopped second, the right angle changed −0.002859 rad while velocity integrated to +0.478576 rad. Reported velocity therefore cannot yet be treated as reliable accumulated wheel rotation.

All 215 ROS joint-state rows matched raw Isaac wheel positions and velocities at a collection offset of +1/60 simulated second (16.667 ms). This is an observed alignment, **not proof of incorrect ROS timestamps**; it does not explain the large velocity-integral discrepancy.

Reduced damping and solver-iteration trials did not resolve the issue; originals were restored. Tiny resting motion alone did not identify a hidden command publisher. Root cause remains unknown.

## Handoff

Next: establish one owner per TF transform, live `odom → base_link → laser_frame` connectivity, consistent simulation time and scan metadata; then proceed toward SLAM. Automate controller activation during bringup. Later, inspect SDK/bridge velocity semantics and tick order before designing another targeted test.

Preserve `commands.csv`, `isaac.csv`, `ros_joint_states.csv`, `ros_odometry.csv`, `report.json`, and `asn_controlled_motion.py` with this report under `results/step7/`. Report status `recorded` indicates collection completed, not acceptance.

[Full session evidence, decision and deferred issue in Notion](https://app.notion.com/p/3ecfc9c738b281f4a6d7c69874e94edd)
