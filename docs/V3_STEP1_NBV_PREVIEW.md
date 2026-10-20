# V3 Step 1 — 3×3 NBV preview (no robot movement)

Add `nbv_core.py` and `nbv_preview_node.py` to `ros2_ws/src/asn_semantical_navigation/asn_semantical_navigation/`. Add `test_nbv_core.py` to the package's `test/` directory. Register `nbv_preview_node` in `setup.py` and add `visualization_msgs`, `std_msgs` and `python3-numpy` package dependencies.

## What it does

- Uses `/map`, `/semantic/coverage_map`, and TF `map → base_link`.
- Creates **8 translation** candidates on a **3×3, 0.6 m offset** grid and **4 in-place rotations** (±45°, ±90°).
- Rejects candidates whose **approximate circular footprint or straight-line segment** intersects static map obstacles/unknown cells. This is a conservative proxy, **not yet full Nav2 path feasibility**.
- Predicts 2D camera visibility using occupied-grid ray casting, nominal **70° HFOV**, **3 m usable range**, `100 exp(-d/1.5)` distance confidence.
- Scores each candidate by **predicted coverage-map improvement (m²) − 0.005 × approximate action seconds**. Each cell retains its maximum confidence, matching the current actual coverage node's update rule.
- Publishes only preview topics; never sends robot motion commands.

## Build and run

Use an existing terminal where the ROS 2 Jazzy environment is sourced; simulation, `/map`, TF, and `coverage_node` should already be running.

```bash
cd ~/projects/autonomous-semantic-navigation/ros2_ws
colcon build --symlink-install --packages-select asn_semantical_navigation
source install/setup.bash
ros2 run asn_semantical_navigation nbv_preview_node --ros-args -p use_sim_time:=true
```

In RViz, **Add → MarkerArray** and set topic `/semantic/nbv/candidates`. Green arrow: best current predicted viewpoint; blue arrows: other valid candidates; gray arrows: rejected ones. Add a **Map** for `/semantic/nbv/predicted_coverage_map` (separate from the measured `/semantic/coverage_map`).

Other topics: `/semantic/nbv/best_pose` (`geometry_msgs/PoseStamped`), `/semantic/nbv/status` (`std_msgs/String`). The pose is **advisory only**, never dispatched to Nav2.

To inspect parameters: `ros2 param list /nbv_preview_node`. Set actual camera horizontal FOV when calibrated, e.g. `-p fov_deg:=<verified-value>`. Rotate/drive the robot manually and watch the preview change.

## Boundaries of this milestone

**Not implemented:** Nav2 execution, confidence frontier fallback, YOLO target detection, LLM. The preview is a *planar geometric estimate*, not a guarantee of valid 3D RGB-D rays or object detection; it cannot see furniture height, overhangs, or missing depth returns. Measured coverage still comes only from the original `/depth_pcl` node. No Isaac Sim validation has been performed by this patch.
