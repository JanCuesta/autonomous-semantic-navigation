# Step 8B — caster friction and wheel odometry validation

**Date:** 2026-10-08  
**Scope:** Isaac Sim Create 3 V1, ROS 2 Jazzy, wheel-derived `/odom`.  
**Decision:** Keep the idealized zero-friction caster and effective odometry wheel separation **0.23822 m** for V1 simulation. This is sufficient to proceed; the remaining velocity and heading differences are measured limitations, not a claim of exact motion.

## Method and changes

The Step 8B recorder compared independent Isaac articulation-root pose (`base_link`) with ROS wheel odometry at aligned simulation times. It also recorded the commanded motion and wheel positions. Ground-truth pose was used only for evaluation, never published as navigation odometry. The original campaign included forward/reverse straight motion and three repeated left/right in-place turns at 0.40, 0.75, and 1.10 rad/s.

The robot's front caster is a sphere collider fixed under `base_link`, rather than a separate freely rolling caster joint. We temporarily bound a physics material with zero static/dynamic friction and `min` combine mode to that sphere. This isolated caster drag without changing wheel drives or ROS parameters. The material-binding record confirmed the temporary change. We repeated the angular campaign and ran moving forward-left and reverse-right curves (`v=±0.25 m/s`, `ω=±0.75 rad/s`).

Using the measured wheel rotations and Isaac yaw across 18 in-place turns and two initial curves, we fitted a **provisional odometry-only** separation of 0.23822 m, chosen to reduce the largest absolute endpoint heading error across those runs. We changed `asn_wheel_odom_unwrapped`'s default and `simulation.launch.py` from 0.23993 m to 0.23822 m and verified the live node loaded it. The controller's nominal wheel separation remains 0.233 m; wheel radius remains 0.03575 m. We then repeated six stationary turns and two curve campaigns with the new odometry setting.

Finally, the same zero-friction material was saved in `isaac_sim/robots/V1_Robot.usd` on the test workstation, with a timestamped backup of the previous asset. The anonymous experiment session layer was removed. The user confirmed the material after reopening the world and reported that motion worked. The reopened-world curve summary below is additional evidence; its full JSON report was not supplied.

## Results

| Measurement | Before caster change | With zero-friction caster | With zero-friction caster and 0.23822 m odometry |
|---|---:|---:|---:|
| Actual left/right speed for ±0.40 rad/s request | +0.347 / −0.348 | +0.375 / −0.377 | +0.375 / −0.377 |
| Actual left/right speed for ±0.75 rad/s request | +0.675 / −0.680 | +0.720 / −0.721 | +0.720 / −0.721 |
| Actual left/right speed for ±1.10 rad/s request | +1.007 / −1.024 | +1.070 / −1.072 | +1.070 / −1.072 |
| Worst endpoint odometry heading error among stationary turns | 4.86° in original sweep | 5.13° (old 0.23993 m estimator) | **3.40°** in new six-turn run |

The robot now turns closer to the requested speed and more symmetrically. The odometry separation change improved the stationary heading estimate; it did not alter the robot's actual motion. All six stationary turns in the new run stopped, and their translation-vector errors were about 1–2 mm. Both controllers were active despite a redundant `joint_state_broadcaster` spawner error during one ROS restart.

| Moving-curve run at 0.23822 m | Forward-left heading / position error | Reverse-right heading / position error |
|---|---:|---:|
| First run | +2.46° / 1.57 cm | −3.41° / 2.41 cm |
| Repeat | +0.72° / 0.74 cm | +1.19° / 1.31 cm |
| After reopening saved scene | +2.82° / 1.63 cm | +1.17° / 1.27 cm |

The reopened run measured forward-left `v=0.2508 m/s, ω=0.6981 rad/s` and reverse-right `v=−0.2489 m/s, ω=−0.7267 rad/s`. Its stopping times were 0.358 and 0.425 simulation seconds, respectively. It used the same curve commands, but the CSV alone does not establish controller state, active ROS parameters, or the experiment's analysis status.

## Interpretation and limits

- The caster contributed materially to the original slow, asymmetric turns. Zero friction is an **idealized rolling support** for this fixed sphere, not a measured real caster material.
- Straight-line wheel odometry was already accurate in the original sweep (largest reported position error about 0.31 mm); the controller still tracked linear commands slightly below request. The radius was left unchanged.
- Curved motion varies between runs even with the same commands. An effective wheel separation cannot remove all contact/slip effects. The 0.23822 m value is a practical compromise, not a physical measurement of axle spacing.
- The corrected odometry continues to unwrap discontinuous wheel positions. Isaac pose stays evaluation-only; wheel positions remain the source of `/odom` and `odom → base_link`.
- We did not introduce a command-velocity compensation function. It could be revisited only if a navigation task shows a consistent, material shortfall. A real robot would require its own caster model and wheel calibration.

## Evidence and repository state

The timestamped Step 8B run folders on the workstation contain `report.json`, `summary.csv`, `manifest.json`, and raw CSVs under `results/validation/Step8/`. The key uploaded records were the baseline sweep, zero-friction angular/arc runs, calibrated angular/arc runs, reopened-world `summary.csv`, and `Step8B_caster_apply.json`. The local test scripts and calibration notes are under `tests/validation/Step8/`.

The robot asset save and reopened-world check happened on the workstation. This Markdown report is a separate repo file to copy there; preparing it here does not commit or push those workstation changes. Retain the robot-asset backup until the scene and repo changes are committed. No further Step 8B sweep is needed for the current V1 scope.
