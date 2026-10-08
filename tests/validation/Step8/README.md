# Step 8B: motion and wheel odometry at final V1 dynamics

This is a new baseline campaign. Keep `tests/Step8/` and `results/step8/` as historical evidence. The harness does not change physics, gains, geometry, robot pose, TF ownership, or ROS parameters. Independent Isaac `base_link` pose is recorded only for evaluation, never fed to `/odom` or navigation.

## Baseline

| Setting | Controller | Wheel odometry |
|---|---:|---:|
| Wheel radius | 0.03575 m | 0.03575 m |
| Wheel separation | 0.23300 m | 0.23993 m |

| Limit | Linear | Angular |
|---|---:|---:|
| Velocity | -0.40 to +0.40 m/s | -1.10 to +1.10 rad/s |
| Acceleration | +0.60 m/s² | +1.80 rad/s² |
| Deceleration | -0.80 m/s² | -2.20 rad/s² |

The script reads live controller/estimator geometry and records it separately from the checked-in configuration. It requires the final motion limits above, and refuses to run if live limits differ. It saves configuration text, hashes and Git state in `manifest.json` so later tuning can be compared with this baseline. Keep all settings fixed during a campaign.

## Before running

- Launch/restart Isaac only using your **Isaac Safe launcher**. This script uses the running instance.
- Open `Testing_World+Robot.usd`, press **Play**, and bring up the normal ROS simulation stack with `diff_drive_controller`, `joint_state_broadcaster`, and `asn_wheel_odom_unwrapped` active.
- Stop navigation/teleop/mission command producers. The harness publishes `TwistStamped` directly to `/diff_drive_controller/cmd_vel`; Nav2 and safety/mux behavior are outside this particular test. A second publisher on that topic aborts the campaign.
- Place the robot manually in a clear test area before arming. Motion is open-loop: no obstacle avoidance or automatic pose reset. Forward/reverse and left/right pairs reduce excursion but do not guarantee return to the start. Allow about 1.5 m ahead/behind for the translation sweep and a clear area larger than the 0.75 m square plus the robot footprint. Watch the run; Ctrl+C sends zero commands and preserves evidence.
- Do not pause, reset, teleport, change gains, or alter parameters during recording.

## First run: short smoke test

1. In Isaac's **Script Editor**, open and execute:

   `~/projects/autonomous-semantic-navigation/tests/validation/Step8/asn_step8b_motion_odom_validation.py`

   It should print `STEP 8B ISAAC RECORDER ARMED`. It uses the same validated tensor articulation-root reader as the original Step 8 harness. The recorder waits up to five wall-clock minutes for the terminal driver. Do not arm a second copy while the first is still running.

2. In a ROS-sourced Ubuntu terminal, run:

   ```bash
   cd ~/projects/autonomous-semantic-navigation && source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && python3 tests/validation/Step8/asn_step8b_motion_odom_validation.py --ros --scenario smoke --repeats 1
   ```

   This commands +0.10 and -0.10 m/s for two simulation seconds each, with baseline and settling phases. Review the report before the longer tests. Isaac must keep playing until the recorder finishes.

## Campaigns

Re-arm the Isaac recorder before **each** terminal run. Replace the smoke command's final options with one of these:

| Options | Maneuvers |
|---|---|
| `--scenario linear --repeats 3` | ±0.10, ±0.25, ±0.40 m/s; three seconds at each request |
| `--scenario angular --repeats 3` | ±0.40, ±0.75, ±1.10 rad/s; four seconds at each request |
| `--scenario sweep --repeats 3` | Translation and in-place rotation sweeps |
| `--scenario arc --repeats 3` | (+0.25 m/s, +0.75 rad/s), then (-0.25 m/s, -0.75 rad/s); four seconds each |
| `--scenario square --repeats 3` | Four nominal 0.75 m sides at 0.25 m/s; left turns at 0.75 rad/s |
| `--scenario all --repeats 3` | Sweeps, arc pair, and square in one recording |

Start with separate campaigns so you can inspect results and reposition between them. The full campaign is supported, but requires enough clear space for accumulated drift. Repeats within a recording are consecutive runs without pose resets; they measure repeatability under those conditions, rather than identical initial poses.

Square command durations include a nominal acceleration/braking correction. They are feedforward requests, not closed-loop ground-truth turns. Actual square closure therefore tests physical motion as well as odometry: compare the Isaac endpoint with the start **and** the odometry endpoint with the Isaac endpoint.

All phase durations, distances, yaw, and stopping metrics use **simulation time**. The recorder timeout scales to the campaign, with headroom down to approximately 0.125 real-time factor. Publishing runs at approximately 50 wall Hz; stalls and phase wall deadlines still abort. A long recording cannot expire at the original three-minute limit.

## What the report measures

1. **Command tracking:** requested velocity versus Isaac pose-derived body velocity over the plateau after nominal acceleration plus 0.4 s. Reported ratios show whether the controller/physics achieves the requested speed. A request-time integral is also saved, but omits acceleration and is not a displacement acceptance criterion.
2. **Odometry accuracy:** `/odom` versus independent Isaac pose over identical simulation-time boundaries, including braking. Each source is expressed in its own initial body frame, so different world/odom origins and orientations do not create false errors. Continuous yaw is used for accumulated error; wrapped endpoint heading is reported separately. Turns beyond π and a complete square do not lose revolution information.
3. **Stopping:** time and traveled path after the zero command until GT translation speed is below 0.01 m/s and angular speed below 0.02 rad/s for 0.5 sim s. These are measurement thresholds, not a new project acceptance policy. If not reached during the settle phase, stopping time is null and the recording is flagged incomplete.
4. **Wheel evidence:** unwrapped wheel-position changes and wrap event counts, using the live estimator geometry. Raw wheel velocities/targets remain evidence; the previously deferred velocity-integral issue is not an acceptance test. Wheel evidence is not a second independent pose ground truth.
5. **Repeatability and square closure:** mean, sample standard deviation, range, and whole-square endpoint/accumulated-heading comparisons.
6. **Calibration candidates:** radius candidates from straight runs and separation candidates from in-place turns, with median/range across runs. These are advisory values only. Review command tracking and speed/direction dependence before tuning; investigate slip or asymmetry rather than averaging it away. Radius changes also affect yaw scale, so re-evaluate separation after any radius adjustment.

Interpolation uses bracketing samples, never extrapolation. Source gaps larger than 0.20 sim s, time resets, missing feedback, competing command publishers, or recorder loss are rejected. Wrapped wheel positions are unwrapped before boundary interpolation. As with the estimator, unwrapping assumes physical wheel increments between samples remain below π; sampling gaps and overspeed need attention at higher speeds.

A completed run is `recorded_pending_review`, **not automatically accepted**. This baseline supplies evidence for setting precision tolerances and deciding whether calibration is justified. The script does not validate SLAM, AMCL, Nav2, mission management, or command-loss expiry.

## Results

Each run creates a timestamped folder in:

`~/projects/autonomous-semantic-navigation/results/validation/Step8/`

| File | Contents |
|---|---|
| `manifest.json` | Campaign plan, actual phase boundaries, live parameters, source snapshot, abort reason |
| `commands.csv` | Published requests with simulation and wall timestamps |
| `isaac.csv` | Independent root pose, wheel positions, raw wheel velocities and targets |
| `ros_joint_states.csv` | ROS wheel feedback |
| `ros_odometry.csv` | Estimated pose and twist |
| `finished.json` | Isaac recorder completion status |
| `report.json` | Detailed motion, odometry, stopping, repeats and calibration metrics |
| `summary.csv` | One row per evaluated maneuver |
| `report.md` | Readable overview |

An abort still saves ROS evidence and attempts to finalize the Isaac recorder; files unavailable because of a simulator crash cannot be recovered automatically. Reports mark incomplete data explicitly. Send `report.json`, `summary.csv`, and `manifest.json` for first review; raw CSVs allow deeper diagnosis.

## Offline checks and reanalysis

These commands do not import ROS or Isaac and cannot move the robot:

```bash
python3 tests/validation/Step8/asn_step8b_motion_odom_validation.py --plan --scenario all --repeats 3
python3 tests/validation/Step8/asn_step8b_motion_odom_validation.py --self-test
python3 tests/validation/Step8/asn_step8b_motion_odom_validation.py --analyze results/validation/Step8/REPLACE_WITH_RUN_FOLDER
```

Offline checks cover frame alignment with asynchronous timestamps, reverse motion, multi-revolution yaw, curved path length, ±4π wheel wraps, missing-data rejection, known radius scale errors, stopping, report output and abort status. They verify analysis logic; live ROS/Isaac integration must be checked on your workstation.
