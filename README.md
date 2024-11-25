# AquaBot Challenge - Team ROS2-D2

## Project Overview

This project was completed as part of the **AquaBot Challenge**, where we developed an autonomous system using **ROS 2** to control an Unmanned Surface Vehicle (USV). The primary goal was to accomplish a series of missions around offshore wind turbines, including inspection, stabilization, and navigation in a complex environment.

We were a team of four students, and the entire project was developed in just **one week**.

## Technical Contributions

- **Autonomous Navigation**:
  - Implemented a trajectory planner (Path Planner) to generate optimized paths while avoiding obstacles.
  - Integrated **PID** and **Model Predictive Control (MPC)** for precise control of the USV following planned trajectories.

- **Wind Turbine Inspection**:
  - Detected and tracked wind turbines using `TF` transformations.
  - Used computer vision (OpenCV) to detect and decode QR codes on wind turbines.

- **Stabilization**:
  - Controlled the USV to remain stationary and aligned with a wind turbine upon QR code detection.

- **Task Management**:
  - Developed a **Manager Node** to coordinate mission phases, publish navigation goals, and process sensor data.

## Code Architecture

The project is structured into several ROS 2 nodes:

- **Navigator**: Handles trajectory planning, obstacle avoidance, and path generation around wind turbines.
- **Quartermaster**: Implements camera control and MPC algorithms.
- **Manager**: Coordinates mission phases, publishes goals, and manages the USV's state.
- **Gunner**: Configures and launches all necessary nodes.

## Challenges

- Implementing the **MPC** due to the complexity of transformations and available libraries.
- Ensuring accurate sensor calibration and managing transformation errors between coordinate frames.
- Coordinating a team working simultaneously on different modules within a tight timeline.

## Execution Instructions

To run the complete system:

```bash
ros2 launch gunner board_the_ship.launch.py
