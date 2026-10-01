# ROS 2 Control / Isaac Sim Recovery — 2026-09-30

## Context

This session completed the V1 motion-control check: ROS 2 velocity commands had to reach the simulated differential-drive base in Isaac Sim, produce wheel motion and odometry, and stop after command loss.

## What was happening

After reopening `Testing_World+Robot.usd`, ROS 2 could see `/isaac_sim_control`, but `/controller_manager` did not exist. `ros2 control list_controllers` therefore waited indefinitely.

Isaac's log showed that the ROS 2 Control extension started, but the graph failed when it tried to create the controller manager for `/World/V1_Robot`:

```text
ROS2ControlManager setup failed for '/World/V1_Robot':
setup_cm: ControllerManager init failed
```

This meant ROS discovery itself was working; the failure was inside the ROS 2 Control initialization path.

## Diagnostics performed

| Check | Result | What it proved |
|---|---|---|
| Plain Isaac Sim launch | `/isaac_sim_control` and `/clock` existed, but no `/controller_manager` | The generic Isaac ROS node was not evidence that `ros2_control` had initialized. |
| Safe-launcher inspection | Isaac bundled Jazzy library directory was first in `LD_LIBRARY_PATH` | The launcher still contained the environment fix from the earlier debugging session. |
| Loaded-library inspection | `controller_manager`, `hardware_interface`, `rclcpp`, and `libtinyxml2.so.9` came from Isaac's bundled Jazzy directory | The current failure was not caused by mixing the system ROS controller stack with Isaac's controller stack. |
| `ldd` with the safe-launcher environment | `libsdformat_urdf.so`, `libtinyxml2.so.9`, `libsdformat14.so.14`, and `liburdfdom_model.so.4.0` all resolved inside Isaac's bundled Jazzy directory | The earlier TinyXML / SDF library-resolution concern was ruled out for this run. |
| USD controller configuration inspection | The saved world contained `inputs:controllerConfig = "/tmp/asn_cm_minimal.yaml"` | A temporary diagnostic configuration had been persisted in the world. |
| YAML comparison | `/tmp/asn_cm_minimal.yaml` contained only `update_rate: 50`; the project YAML contained the joint-state and differential-drive controllers plus wheel parameters | The world was not using the intended project controller configuration. |

## Fix

The simulation was stopped and the ROS2ControlManager `Controller Config` was restored to the real, absolute project path:

```text
/home/master-jan/projects/autonomous-semantic-navigation/ros2_ws/src/asn_simulation/config/ros2_controllers.yaml
```

The world was then saved and simulation restarted.

After that, ROS 2 reported:

```text
/controller_manager
/isaac_sim_control
/v1_robot
```

The controller manager was therefore initializing correctly again.

The controllers were then loaded and activated:

```text
diff_drive_controller   diff_drive_controller/DiffDriveController      active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active
```

## Motion test

A stamped velocity command of `0.05 m/s` forward was published to:

```text
/diff_drive_controller/cmd_vel
```

During motion, the wheel-state feedback showed approximately:

```text
left_wheel_joint  = 1.910 rad/s
right_wheel_joint = 2.003 rad/s
```

Odometry reported:

```text
x         = 0.1021 m
linear.x  = 0.04996 m/s
angular.z = -0.000297 rad/s
```

This verified the command path:

```text
TwistStamped command
    -> diff_drive_controller
    -> wheel velocity interfaces
    -> simulated wheel motion
    -> wheel-derived odometry
```

The very small residual angular velocity was not treated as a blocker; straight-line accuracy belongs to the later odometry-validation work.

## Command-expiry test

The forward command was published for two seconds and then removed. After a one-second wait, odometry reported approximately:

```text
linear.x  = -0.0000435 m/s
angular.z = -0.000374 rad/s
```

These values are effectively zero for this simulation check. The robot therefore stopped after the command source disappeared, confirming the configured command-timeout behavior.

## Conclusion

The immediate restart failure was caused by the saved Isaac world still referencing the temporary diagnostic controller YAML instead of the project's real controller configuration.

Restoring the absolute project YAML path allowed `controller_manager` to initialize again.

The safe launcher should be kept as-is: its bundled Jazzy library ordering was checked and was resolving the relevant ROS 2 Control dependencies consistently.

The session verified, end to end, that ROS 2 velocity commands reach the simulated wheels, wheel feedback is available, odometry responds correctly, and command expiry stops the robot.

This is a successful functional verification of the V1 motion/control step. It is not yet the full repeated acceptance campaign.

## Next step

Continue with sensor/time/frame integration and validation: LiDAR, simulation time, TF, and odometry consistency. Also automate controller loading during bringup so a clean restart does not require manual `ros2 control load_controller` commands.
