# Drone Network for Search and Rescue (SAR) System

A ROS 2–based multi-UAV Search and Rescue (SAR) system designed for structured indoor environments under realistic communication and computational constraints.

The system integrates decentralized coordination, real-time human detection, and obstacle-aware navigation into a unified architecture validated in simulation and on DJI Tello EDU drones.

## Key Features

- Deterministic region-based decentralized coordination (no central task allocator)
- Finite State Machine (`SEARCHING`, `ASSISTING`, `VERIFYING`, `IDLE`)
- YOLO-based real-time human detection
- 2D occupancy grid mapping
- A*-based obstacle-aware path planning
- Detection-triggered state transitions
- Sim-to-real validation (Gazebo → Tello EDU)

## Architecture Overview

Each UAV runs three logical modules:

1. **Coordination**  
   Region partitioning and workload redistribution.

2. **Detection**  
   Lightweight YOLO-based human detection with multi-frame validation.

3. **Navigation**  
   Incremental occupancy grid updates with A* planning and dynamic obstacle handling.

High-level computation runs on a Ground Control Station, while decision logic remains decentralized through ROS 2 topic exchange.

## Evaluation Metrics

- Coverage efficiency
- Time-to-detection
- Coverage overlap
- Collision-free mission completion
- Replanning latency
- Sim-to-real consistency

## Technologies

- ROS 2 Humble
- Gazebo
- Python
- YOLO (single-stage detection)
- A* path planning
- DJI Tello EDU SDK

## Academic Context

Developed as part of DASE 4460 / CS 3460 (Spring 2026).

## License

MIT License
