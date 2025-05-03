# Obstacle Avoidance Robot Project

This repository contains the implementation of an obstacle avoidance robot system using ROS (Robot Operating System). The project integrates multiple packages for control, mapping, simulation, and dynamic object detection to enable autonomous navigation in dynamic environments.

## Project Overview

The project is structured into the following key packages:

1. **Control**: Provides low-level motor control, power management, and odometry updates for the robot.
2. **GMPC Controller**: Implements a Generalized Model Predictive Control (GMPC) algorithm for trajectory tracking and obstacle avoidance.
3. **Map Manager**: Implements 3D mapping algorithms, including occupancy voxel maps, ESDF maps, and dynamic maps for real-time navigation.
4. **Onboard Detector**: Detects and tracks dynamic obstacles using RGB-D cameras.
5. **RealSense Node**: Publishes RealSense camera data, including color and depth images.
6. **Simulations**: Simulates the robot's behavior, including path planning, visualization, and data recording.

---

## Installation

### Prerequisites
- ROS (Melodic/Noetic)
- Python 3
- Required libraries: `numpy`, `shapely`, `pandas`, `pyrealsense2`, `cv_bridge`, `Eigen`, `qpOASES`

### Steps
1. Clone the repository into your ROS workspace:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/Squawk8284/MTECH_obstacle_avoidance_robot.git
    cd ~/catkin_ws
    catkin_make
    ```

2. Install additional dependencies:
    ```bash
    sudo apt-get install ros-<distro>-vision-msgs ros-<distro>-cv-bridge ros-<distro>-sensor-msgs ros-<distro>-geometry-msgs
    pip install numpy shapely pandas pyrealsense2
    ```

---

## Packages Overview

### 1. **Control**
- **Description**: Handles motor control, power management, and odometry updates.
- **Key Features**:
  - Supports open-loop, closed-loop, and position control modes.
  - Publishes odometry data and subscribes to velocity commands.
- **Launch**:
    ```bash
    roslaunch control control.launch param_file:=<path_to_config.yaml>
    ```

### 2. **GMPC Controller**
- **Description**: Implements a Nonlinear Model Predictive Control (NMPC) algorithm for trajectory tracking and obstacle avoidance.
- **Key Features**:
  - Tracks reference paths and avoids obstacles.
  - Uses `qpOASES` for solving quadratic programming problems.
- **Launch**:
    ```bash
    roslaunch gmpc_controller gmpc.launch
    ```

### 3. **Map Manager**
- **Description**: Provides 3D mapping capabilities, including occupancy maps, ESDF maps, and dynamic maps.
- **Key Features**:
  - Supports collision checking and distance field computation.
  - Integrates dynamic obstacle detection for real-time navigation.
- **Launch**:
    ```bash
    roslaunch map_manager occupancy_map.launch
    ```

### 4. **Onboard Detector**
- **Description**: Detects and tracks dynamic obstacles using RGB-D cameras.
- **Key Features**:
  - Supports real-time detection on resource-constrained devices.
  - Publishes dynamic obstacle positions and velocities.
- **Launch**:
    ```bash
    roslaunch onboard_detector run_detector.launch
    ```

### 5. **RealSense Node**
- **Description**: Publishes RealSense camera data, including color and depth images.
- **Key Features**:
  - Publishes aligned and raw depth images.
  - Converts camera data to ROS topics for integration with other packages.
- **Launch**:
    ```bash
    rosrun realsense_node realsense_publisher.py
    ```

### 6. **Simulations**
- **Description**: Simulates the robot's behavior, including path planning, visualization, and data recording.
- **Key Features**:
  - Implements TLBO-based path planning.
  - Publishes planned and actual paths for visualization in RViz.
  - Records simulation data for analysis.
- **Launch**:
    ```bash
    rosrun simulations Navigation.py
    rosrun simulations plot_actual_path.py
    rosrun simulations Plots.py
    ```

---

## Running the Full System

To launch the entire system, use the provided `run.launch` file:
```bash
roslaunch run.launch
```
This will load the required parameters and include launch files for all packages.

---

## Outputs

- **Planned Path**: Published as a `Path` message.
- **Actual Path**: Published as a `Path` message and visualized in RViz.
- **Dynamic Obstacles**: Published as positions and velocities.
- **Data Logs**: CSV files containing time-stamped robot data for analysis.

