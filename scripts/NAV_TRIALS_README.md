# Supervised Nav2 hardware trial recorder

**Laptop only**, ROS 2 Jazzy, TurtleBot 4 on the floor, with *accurate* map localization and a person ready to stop the robot independently of Wi-Fi. **Do not use on the desk or dock.** Full semantic mission must remain `IDLE`; do not run semantic and ordinary navigation goals concurrently.

## Install

Download `nav_trial_runner.py` and `nav_test_goals.json` from this conversation into your repository's `scripts/` directory. The script does not change your existing launch files. In an Ubuntu terminal:

```bash
cd ~/projects/autonomous-semantic-navigation
source ros2_ws/hardware_env.bash
python3 scripts/nav_trial_runner.py --help
```

Keep your existing *single* Nav2 + sensor + semantic supervisor and RViz open. Check battery, `/scan`, `/odom`, `/amcl_pose`, `map -> base_link` TF, collision monitor, E-stop/independent stop, command ownership and that the robot is stationary and physically on the floor. Initial pose at map `(0,0,0)` is valid only if true physically; check LiDAR/map alignment.

## Configure fixed goals

Edit `scripts/nav_test_goals.json`, replacing `null` entries with **map-frame** numeric coordinates of three *physically verified*, safely reachable floor positions; set `enabled` to `true` for each verified goal. **No goals are supplied or pre-approved by the template** because map origin alone does not identify safe destinations. Use RViz map/costmaps and physical clearances, especially turning and reverse clearances. Small displacement alone does not make a goal safe.

## Plan only — no robot motion

```bash
cd ~/projects/autonomous-semantic-navigation
source ros2_ws/hardware_env.bash
python3 scripts/nav_trial_runner.py --goals scripts/nav_test_goals.json
```

The tool requests a Nav2 ComputePathToPose plan for each configured goal and saves `goal.json`, `plan.csv`, `plan_result.json` under `results/nav_trials/<session>/<goal>/`. This is an actual Nav2 planner request but **not a navigation goal**. It requires current scan/odom freshness and map TF. Inspect each returned path in RViz and against the real room; a planner success alone is not physical clearance proof.

## Execute one supervised trial at a time — ROBOT MOTION

Only after all safety gates have passed and the operator/lab approves **each** goal:

```bash
python3 scripts/nav_trial_runner.py --goals scripts/nav_test_goals.json --execute
```

The tool checks active Nav2 lifecycle nodes, fresh `/scan` and `/odom`, map TF, `IDLE` semantic mission, NBV idle, no observed active Nav2 goal, and a fresh, valid battery sample of at least 20%. Then it replans before every goal and asks you to type `MOVE NAV01` etc. **This launches movement**. It does *not* provide an independent E-stop. `Ctrl+C` requests goal cancellation if a goal is already accepted, but cancellation is not a guaranteed stop. Have a second person at the robot with an independent stop method. Do not release stops without an approved procedure.

Each trial saves:
- `goal.json` — commanded map X/Y/yaw
- `plan.csv` and `plan_result.json` — no-motion planned route
- `trajectory.csv` — approximately 4 Hz TF map pose, AMCL pose, odom, velocities, sensor ages and Nav2 remaining-distance feedback
- `result.json` — command, start/final estimated map pose, numeric XY/yaw errors, Nav2 status, odom stationary snapshot, operator confirmation
- `summary.csv` — one row per completed attempted trial in the session.

**Caution:** final pose/position error is **estimated from map TF**, not tape-measured physical ground truth. Write down the physical error separately when measuring floor markers. A Nav2 success or zero odom speed is insufficient to prove safe stopping. The tool stops its sequence if a trial fails or is not physically confirmed; stale scan/odom causes a cancel attempt (not a physical stop guarantee). It never begins the next movement automatically without operator approval.

## Evidence and final validation

After the robot is physically stopped, run:

```bash
scripts/lab_diagnostics.py evidence
```

This is supplemental evidence; the Nav2 trial runner's result files are the primary per-goal records. Keep the launch log too. Before trusting hardware operation, have the terminal-connected agent inspect this new script, run it in plan-only mode with actual ROS 2 Jazzy, and review any errors. Planning was tested on the connected robot; physical execution still requires a supervised hardware test.
