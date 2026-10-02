# Step 8 — Wheel Odometry Validation Report

**Project:** Autonomous Semantic Navigation Robot — V1  
**Date:** 2026-10-02  
**Step:** 8 — Validate wheel odometry  
**Status:** **COMPLETE — validation passed with documented limitations**

## 1. Objective

Validate wheel-derived odometry against an independent Isaac Sim ground-truth pose while keeping simulator ground truth completely separate from the navigation estimate.

The Step 8 campaign covered:

- straight translation;
- reverse translation;
- controlled left and right turns;
- a multi-segment square trajectory;
- translation and heading error measurement;
- a controlled wheel-geometry perturbation;
- TF/source ownership verification.

All durations and comparisons used **simulation time**.

## 2. Final odometry architecture

Isaac Sim 6.1 exposes continuous wheel-joint positions that were observed to jump by approximately ±4π during longer trajectories. With `diff_drive_controller` configured for `position_feedback: true`, those discontinuities were interpreted as real wheel motion and corrupted its odometry.

The V1 simulation solution is therefore:

- `diff_drive_controller`
  - continues to own wheel velocity actuation;
  - `enable_odom_tf: false`;
  - its built-in odometry is not used as the V1 navigation odometry source.
- `asn_wheel_odom_unwrapped`
  - subscribes to `/joint_states`;
  - unwraps each wheel increment sample-to-sample;
  - integrates differential-drive wheel odometry;
  - publishes `/odom`;
  - publishes the sole `odom -> base_link` TF.
- Isaac articulation-root pose
  - corresponds to `/World/V1_Robot/base_link`;
  - is used only for evaluation;
  - is never published into the navigation localization chain.

Final nominal geometry used by the estimator:

- wheel radius: **0.03575 m**
- effective wheel separation: **0.23993 m**

This is an intentional V1 simulation architecture revision. The earlier architecture assumption that `diff_drive_controller` itself owns `/odom` and `odom -> base_link` is superseded for V1 simulation by the unwrapped wheel-odometry node.

## 3. Evidence

### 3.1 Straight motion

A 2 s forward command at 0.05 m/s followed by settling produced approximately:

| Source | Forward displacement |
|---|---:|
| Isaac ground truth | 90.730 mm |
| ROS wheel odometry | 90.728 mm |
| Wheel-position kinematics | 90.729 mm |

The initial report showed an odometry-vs-ground-truth difference of only about **0.002 mm**. A later analysis using a strictly common sampled time window still gave only about **0.015 mm (~0.016%)** error.

Result: **pass**.

### 3.2 Reverse motion

The reverse trial produced:

| Source | Forward displacement |
|---|---:|
| Isaac ground truth | -94.507 mm |
| ROS wheel odometry | -94.612 mm |
| Wheel-position kinematics | -94.612 mm |

Measured odometry error against Isaac:

- translation: approximately **0.105 mm (~0.11%)**
- heading: approximately **0.0018°**

Result: **pass**.

### 3.3 Continuous-joint wrap defect

The first long square test exposed discontinuities of approximately ±4π in wheel position. Examples included wheel-angle deltas such as:

- right wheel: approximately -10.020 rad, equivalent to +2.546 rad after adding 4π;
- right wheel: approximately -7.818 rad, equivalent to +4.748 rad after adding 4π;
- left wheel: approximately +7.743 rad, equivalent to -4.823 rad after subtracting 4π.

The built-in position-feedback odometry integrated these discontinuities as real motion, producing large false pose changes.

A simulation-only unwrapped wheel-position estimator was added. In the corrected final square campaign, the evaluator recorded **3 left-wheel wrap events and 2 right-wheel wrap events**, yet navigation odometry remained coherent.

Result: **root cause isolated and mitigated for V1 simulation**.

### 3.4 Turns

With unwrapped wheel positions and the effective 0.23993 m wheel separation, controlled turns produced approximately:

| Maneuver | Isaac heading change | Odom heading change | Heading error |
|---|---:|---:|---:|
| Left turn | +86.658° | +87.600° | +0.942° |
| Right turn | -87.917° | -86.381° | +1.536° |
| Square turn 1 | +86.690° | +87.641° | +0.952° |
| Square turn 2 | +87.364° | +88.356° | +0.991° |
| Square turn 3 | +86.993° | +88.000° | +1.008° |
| Square turn 4 | +86.626° | +87.636° | +1.009° |

The remaining mismatch is systematic wheel-odometry error, not the earlier wrap failure.

Result: **pass with residual calibration/drift documented**.

### 3.5 Straight segments within the square

Across the four square sides, each physical segment was about 96.7–96.8 mm according to Isaac. Odometry translation-vector error was approximately:

- side 1: **2.073 mm**
- side 2: **2.077 mm**
- side 3: **2.095 mm**
- side 4: **2.098 mm**

Per-side heading error was approximately **0.92–0.93°**.

Result: **coherent and repeatable local odometry**.

### 3.6 Whole-square accumulated error

For the complete square campaign:

- Isaac endpoint displacement from square start: **24.23 mm**
- wheel-odometry endpoint displacement from square start: **15.33 mm**
- odometry-vs-ground-truth endpoint translation-vector error: **9.06 mm**
- Isaac net heading change: **-22.01°**
- odometry net heading change: **-14.32°**
- accumulated heading error: **7.69°**

The physical robot itself did not execute a geometrically perfect square; therefore the test compares odometry with the actual Isaac trajectory, not with ideal commanded geometry.

This accumulated drift is expected evidence of the limitation of wheel-only dead reckoning and motivates later SLAM/localization correction.

Result: **pass with accumulated drift documented**.

## 4. Controlled wheel-geometry sensitivity

The final recorded wheel measurements were replayed offline using the nominal wheel separation and a deliberately incorrect **+5% wheel separation**.

Nominal separation: **0.239930 m**  
Perturbed separation: **0.251926 m**

Examples:

| Maneuver | GT | Nominal error | +5% separation error |
|---|---:|---:|---:|
| Left turn | +86.658° | +0.942° | -3.229° |
| Right turn | -87.917° | +1.536° | +5.649° |
| Square turn 1 | +86.690° | +0.952° | -3.222° |
| Square turn 2 | +87.364° | +0.991° | -3.216° |
| Square turn 3 | +86.993° | +1.008° | -3.183° |
| Square turn 4 | +86.626° | +1.009° | -3.164° |
| Whole square | -22.006° | +7.688° | -8.773° |

This demonstrates that heading accuracy is materially sensitive to wheel-geometry calibration.

Result: **controlled geometry-error requirement satisfied**.

## 5. Final interface / TF verification

Final runtime verification showed:

- `/odom`
  - type: `nav_msgs/msg/Odometry`
  - **publisher count: 1**
  - publisher: `asn_wheel_odom_unwrapped`
- `/diff_drive_controller enable_odom_tf`
  - **False**
- `/asn_wheel_odom_unwrapped publish_tf`
  - **True**
- `odom -> base_link`
  - available from TF and updating in simulation time.

`tf2_echo` initially reported that `odom` was not yet in its local buffer, then successfully received the transform. This is a startup/cache observation, not a persistent TF failure.

Result: **single navigation odometry and TF ownership verified**.

## 6. Deferred / known limitations

1. **Raw wheel velocity inconsistency remains unresolved.**  
   Step 7 showed that raw wheel velocity integration, particularly on the right wheel, can disagree with measured wheel-angle change. Step 8 deliberately uses wheel position increments and does not treat raw velocity integration as an acceptance source.

2. **V1 wheel odometry is simulation-specific.**  
   `asn_wheel_odom_unwrapped` exists to handle Isaac continuous-joint position wrapping. V2 should use the physical robot's encoder/base-driver odometry and be separately calibrated and validated.

3. **Wheel-only odometry drifts.**  
   The square campaign demonstrates accumulated translation and heading drift. Later SLAM/localization must correct this; ground truth must not be substituted as navigation odometry.

4. **The effective separation is calibrated rather than identical to the nominal geometric separation.**  
   The simulated odometry currently uses 0.23993 m rather than the original 0.233 m.

5. **`base_link -> laser_frame` static TF was not repaired in Step 8.**  
   It is required before SLAM integration, but it is not a prerequisite for the independent wheel-odometry-vs-Isaac measurements completed here.

6. **Controller/node startup is not yet fully automated.**  
   Controller activation and the custom odometry node should be integrated into bringup before normal mapping/navigation workflows.

## 7. Acceptance decision

**Step 8 is COMPLETE.**

The evidence demonstrates that:

- navigation odometry is wheel-derived rather than perfect simulator global pose;
- Isaac ground truth remains evaluation-only;
- straight, reverse, turning and square motion were quantitatively compared;
- translation and heading errors were measured;
- long-run wheel-position wrapping was detected and mitigated;
- controlled wheel-geometry error was tested;
- a single `/odom` publisher and single `odom -> base_link` TF owner were verified;
- remaining limitations and estimate sources are documented.

The next software-plan activity is **Step 9 — implement mapping with SLAM Toolbox**. Before mapping acceptance, bringup should start the corrected wheel odometry automatically and restore the required static robot/sensor TF chain.
