# Upload your real ASN demonstration media

The root README is prewired to the following asset paths. **The `.png` and `.mp4` files in this bundle are labeled placeholder assets, not project screenshots or demonstrations.** Replace each placeholder with a capture from the real project using the exact same filename. This is the only media-editing step needed.

## Priority captures

| File to replace | Actual media to upload |
|---|---|
| `media/screenshots/isaac-sim-overview.png` | Wide, clean shot of the robot in the current Isaac Sim world |
| `media/screenshots/rviz-navigation.png` | RViz with occupancy map, robot pose, LiDAR and a live actual planned path |
| `media/videos/navigation-demo.mp4` | 30–60 sec real navigation attempt; ideally show Isaac Sim and RViz together |

Optional: `media/videos/mission-manager-demo.mp4` (readiness, cancellation or a real recovery case), `media/screenshots/validation-results.png` (measured quantitative chart).

## Suggested capture sequence

1. Launch the configured Isaac Sim and ROS 2 stack with the **actual current map**.
2. Show the simulation world and RViz side by side.
3. Show **initial pose** and **goal selection**; record the planned path only if actually available.
4. Keep recording through success/failure; capture mission-manager state, terminal output and time.
5. If an attempt fails or remains incomplete, label it a **development demo**, not a successful autonomous mission.
6. Crop any private desktop content; avoid unlicensed music or copyrighted external imagery.
7. Use 16:9 PNG images where possible. Compress demo MP4 enough for hosting; GitHub restricts large file uploads.

**If you host video externally** instead of committing an MP4, replace the README MP4 link with a YouTube/other video URL. Otherwise the exact filenames above need no README edits.

## Evidence principles

A video of a single success does not establish reliability. Document the test setup, simulation versus wall-clock timing and any limitations. Actual evidence and numerical reports belong in `results/`, not this media directory.
