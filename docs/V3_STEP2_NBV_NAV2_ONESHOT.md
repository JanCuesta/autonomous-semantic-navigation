# V3 Step 2 — Send exactly one NBV action through Nav2

**Status:** Code and local unit tests only. The ROS 2 Jazzy/Isaac Sim runtime has **not yet been exercised** in this environment. This is a *simulation-first, explicitly operator-triggered* experiment, **not an autonomous search loop**.

## Goal and architecture

Use the Step 1 3×3 NBV preview scorer without changing its geometry or confidence assumptions. After an explicit Trigger service call, compute the current best safe candidate, send **one** `nav2_msgs/action/NavigateToPose` goal, wait for Nav2 to finish, briefly wait for the RGB-D coverage stream, and print an actual-vs-predicted coverage-gain comparison. **Do not** choose a second destination automatically.

```
/map + /semantic/coverage_map + map->base_link TF
       -> Step 1 NBV candidate ranking (8 translations, 4 rotations)
       -> /semantic/nbv/execute_once (operator service)
       -> /navigate_to_pose (Nav2 action)
       -> Nav2 result + final pose + coverage map difference
       -> STOP, await another explicit Trigger
```

Safety: No movement at node startup. Active Nav2/mission goals must not compete with this test. Nav2 still owns path planning, obstacle avoidance, and controller safety; Step 1's footprint/straight-line checks are only preliminary filters. `cancel` service and goal timeout are provided. Test in Isaac Sim before using real hardware.

## Files changed (overlay for the already-installed Step 1)

- Added `asn_semantical_navigation/nbv_execute_once_node.py` (ROS action client and one-shot state machine).
- Added `asn_semantical_navigation/nbv_execution_math.py` and `test/test_nbv_execution_math.py`.
- Small backwards-compatible extension of `nbv_preview_node.py` constructor to allow a separate executor node name (the original `nbv_preview_node` executable still behaves as before).
- Updated `setup.py` to register `nbv_execute_once_node` and `package.xml` for `nav2_msgs`, `action_msgs`, and `std_srvs`.
- **Not changed:** `nbv_core.py`, `coverage_node.py`, Nav2 configuration, Isaac Sim or mission manager.

## Install

Assuming the user installed Step 1 and the ZIP is in `~/Downloads`:

```bash
cd ~/projects/autonomous-semantic-navigation
unzip -o ~/Downloads/v3_nbv_step2_overlay.zip -d .
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select asn_semantical_navigation
source install/setup.bash
```

## Run one action

Ensure your usual Isaac Sim, `asn_bringup` navigation (AMCL), coverage node, and `map -> base_link` TF are working. Initialize your pose in RViz. **Stop `nbv_preview_node` and avoid simultaneously submitting RViz, mission-manager or other navigation goals.** Open two sourced ROS terminals.

**Terminal A:**

```bash
ros2 run asn_semantical_navigation nbv_execute_once_node --ros-args -p use_sim_time:=true
```

**Terminal B (run just once):**

```bash
ros2 action list -t | grep navigate_to_pose
ros2 service call /semantic/nbv/execute_once std_srvs/srv/Trigger '{}'
```

The service returning `success: true` means **the action request was dispatched**, **not that Nav2 succeeded**. Wait for `NAV2_SUCCEEDED`, `NAV2_ABORTED`, `NAV2_CANCELED`, or a navigation error followed by the final summary. To watch state changes:

```bash
ros2 topic echo /semantic/nbv/execution_status
```

To request cancellation if something goes wrong:

```bash
ros2 service call /semantic/nbv/cancel std_srvs/srv/Trigger '{}'
```

`Ctrl+C` also requests a best-effort cancel but a dedicated cancel service is preferable. If the navigation system continues moving unexpectedly, use your existing ROS/simulation safety-stop procedure. Only issue another `/execute_once` after the final `READY for another EXPLICIT manual trigger` message.

## Expected validation

Typical *format* (numbers below are illustrative, not tested motion results):

```
REQUESTED turn(-90) -> (...), predicted gain=0.868 m2
NAV2_ACCEPTED: executing one selected viewpoint
NAV2_SUCCEEDED: waiting briefly for actual camera coverage update
NAV2_SUCCEEDED | predicted gain=0.8680m2 | actual coverage gain=...m2 | goal error=...m/...deg; pose_check=PASS/FAIL | READY for another EXPLICIT manual trigger
```

The preview topic `/semantic/nbv/candidates` still visualizes the current best candidates. The predicted map topic remains `/semantic/nbv/predicted_coverage_map` (RViz: use **Volatile** durability, as in Step 1).

**Do not expect predicted and actual gains to match exactly.** Predicted visibility uses planar occupancy geometry and the endpoint camera heading; actual coverage uses 3D RGB-D rays received during motion/rotation and can include view angles the predictor omitted. The camera field of view default is an uncalibrated **70°**; the measured `coverage_node` uses `100 exp(-d / 1.5)` for valid readings within 3 m. A measured gain near zero may indicate missing depth updates rather than a failure of the Nav2 action.

## Troubleshooting

| Symptom | Interpretation / check |
|---|---|
| `WAITING: /map or /semantic/coverage_map missing` | Start map server, localization and coverage node. |
| `WAITING: map -> base_link transform unavailable` | In RViz set initial pose and check AMCL / TF. |
| `Nav2 action server unavailable` | `ros2 action list -t`, Nav2 lifecycle states, and simulation startup. |
| `LOCAL_EXHAUSTED` | All nearby candidates fail safety or minimum utility; **frontier fallback not implemented yet**. |
| `NAV2_ABORTED` | Inspect Nav2 controller/planner/costmap logs; current pose or costmap may not agree with the static-map candidate. |
| `NAV2_SUCCEEDED` but `pose_check=FAIL` | Validate final TF, yaw goal tolerance and Nav2 goal checker configuration. |
| No measured coverage gain | Check `/depth_pcl`, `/semantic/coverage_map` and camera TF; allow more `result_settle_s` if needed. |
| Navigation takes longer than 45 wall seconds | Increase `-p goal_timeout_s:=90.0` when launching the node. |

## Local checks performed

The pure-Python geometry/scoring suite and new action-selection/coverage/goal-error tests: **9 passed**. All Step 2 Python files passed `compileall`. Actual ROS Nav2 interaction must be validated on the user's machine.

## Scope intentionally deferred

No repeated autonomous goal loop, no confidence-frontier planner, no AI goal-selection, no YOLO, no intermediate rotation-gain integration. Those follow once **a single selected action** reliably reaches its target and changes the real visual-confidence map.
