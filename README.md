# Autonomous Semantic Navigation Robot

Development of an indoor autonomous mobile robot using ROS 2, C++, Nav2,
SLAM, and NVIDIA Isaac Sim.

## Project Goals

The system will progressively demonstrate:

- ROS 2 software architecture in C++
- Autonomous mapping and localization
- Nav2-based autonomous navigation
- Simulation in NVIDIA Isaac Sim
- Sim-to-real deployment
- Sensor fusion
- RGB-D semantic perception
- Quantitative robotics testing and validation

## Development Stages

### V1 — Simulation
Build and validate autonomous mapping, localization, and navigation in simulation.

### V2 — Physical Robot
Deploy the autonomy stack to a real mobile robot and address sim-to-real differences.

### V3 — Semantic Navigation
Add RGB-D perception and generate navigation goals from detected objects.

## Repository Structure

- `ros2_ws/` — ROS 2 workspace and custom packages
- `isaac_sim/` — simulation worlds, robot assets, and project scripts
- `docs/` — architecture and technical documentation
- `tests/` — system-level tests
- `results/` — experiment and validation results

## Status

Early development / environment setup.
