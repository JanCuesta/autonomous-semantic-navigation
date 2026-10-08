# Step 8B caster experiment: provisional odometry separation

The October 8 zero-friction caster experiment improved actual turning speed.
The material binding was applied to `/World/V1_Robot/base_link/collisions/mesh_2`
in an **anonymous Isaac session layer**. It is not saved in the USD scene.
Keep that layer active when testing this calibration; reverting it invalidates
the calibration. Do not assume that zero friction is a realistic final caster.

Evidence: `report(9).json` (18 in-place turns, three per speed/direction),
`report(10).json` (one forward-left and one reverse-right moving arc), and
`Step8B_caster_apply.json`. All 20 maneuvers completed without analysis errors.
The active odometry radius was 0.03575 m and separation was 0.23993 m.
The controller's nominal separation is 0.233 m and is **not** changed here.

For each maneuver, the measured wheel-position difference supplies its old
odometry yaw `W`, and independent Isaac root pose supplies actual yaw `G`.
At a candidate separation `b`, predicted odometry yaw is
`W * 0.23993 / b`. Minimize the **largest absolute endpoint heading error**
across all 20 maneuvers, in degrees. The exact minimax fit is 0.23821674 m,
rounded to **0.23822 m** in the odometry launch parameter and script default.
This criterion gives the two moving arcs a voice even though there are many
more repetitions of stationary turns. It only fits endpoint heading; moving
arc position must be checked in the live validation.

| Odometry separation | Worst stationary heading error | Forward-left arc | Reverse-right arc | Worst heading error overall |
|---|---:|---:|---:|---:|
| 0.23993 m, previous | 5.13° | -2.07° | -2.23° | 5.13° |
| 0.23822 m, provisional | about 3.40° | about -0.90° | about -3.40° | about 3.40° |

Sign convention is odometry yaw minus Isaac yaw. The stationary-only least
squares fit is approximately 0.23553 m, but it predicts about -5.27° error
on the reverse arc. The arc-only fit is approximately 0.24006 m and leaves
large stationary-turn errors. A single separation cannot make all maneuvers
exact. Changing `b` changes odometry, not the robot's actual turning speed.

## Minimum live verification

With the caster experiment still active, restart the usual ROS launch so the
odometry process reads the new parameter. Isaac stays on the user's Safe
launcher path. Check `/asn_wheel_odom_unwrapped` reports `b=0.23822000` and
that `/odom` still has one publisher. For each command below, re-arm the
existing Step 8B recorder in the Isaac Script Editor before running it:

```bash
cd ~/projects/autonomous-semantic-navigation && source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && python3 tests/validation/Step8/asn_step8b_motion_odom_validation.py --ros --scenario angular --repeats 1
```

```bash
cd ~/projects/autonomous-semantic-navigation && source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && python3 tests/validation/Step8/asn_step8b_motion_odom_validation.py --ros --scenario arc --repeats 1
```

Compare `report.json` and `summary.csv` for both runs with the predictions.
Also inspect arc translation error and stopping behavior. The previous arc
test had 1.6-2.0 cm endpoint translation error and stopped in 0.36-0.44
simulation seconds. These prior position numbers cannot simply be scaled by
the separation ratio. Only adopt the calibration after live verification.
