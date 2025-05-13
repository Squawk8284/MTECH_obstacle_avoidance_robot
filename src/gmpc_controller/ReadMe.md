# GMPC Controller

## Overview

The `gmpc_controller` package implements a Generalized Model Predictive Control (GMPC) algorithm for obstacle avoidance and trajectory tracking in mobile robots. It uses Nonlinear Model Predictive Control (NMPC) to compute optimal control commands for a unicycle robot model.

## Features

- **Trajectory Tracking**: Tracks a reference path provided as a ROS `nav_msgs/Path` message.
- **Obstacle Avoidance**: Ensures smooth navigation by penalizing deviations from the desired trajectory.
- **Nonlinear Dynamics**: Models the robot's unicycle dynamics for accurate control.
- **QP Solver**: Uses `qpOASES` to solve the quadratic programming problem for optimal control inputs.

## Dependencies

This package depends on the following libraries and ROS packages:

- ROS (Robot Operating System)
- `nav_msgs` (for odometry and path messages)
- `geometry_msgs` (for velocity commands)
- `tf` (for orientation calculations)
- `Eigen` (for matrix operations)
- `qpOASES` (for quadratic programming)

## Parameters

The following ROS parameters can be configured:

- `/cmd_vel_topic` (default: `/cmd_vel`): Topic to publish velocity commands.
- `/odom_topic` (default: `/odom`): Topic to subscribe to odometry data.
- `/path_topic` (default: `/path_topic`): Topic to subscribe to the reference path.

## How It Works

1. **Odometry Callback**: Updates the robot's current state and computes control commands if a valid path is available.
2. **Path Callback**: Updates the reference trajectory for the robot to follow.
3. **NMPC Algorithm**:
    - Linearizes the robot's dynamics around a predicted trajectory.
    - Solves a quadratic programming problem to compute optimal control inputs.
    - Publishes the computed velocity commands to the `/cmd_vel` topic.

## Usage

1. Clone the repository into your ROS workspace:
    ```bash
    cd ~/catkin_ws/src
    git clone <repository_url>
    ```
2. Build the package:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
3. Launch the controller:
    ```bash
    rosrun gmpc_controller gmpc_controller
    ```
4. Publish a reference path to the `/path_topic` and ensure odometry data is available on the `/odom` topic.

## Topics

- **Subscribed Topics**:
  - `/odom` (`nav_msgs/Odometry`): Provides the robot's current state.
  - `/path_topic` (`nav_msgs/Path`): Provides the reference trajectory.

- **Published Topics**:
  - `/cmd_vel` (`geometry_msgs/Twist`): Publishes the computed velocity commands.

## Tuning Parameters

The following parameters can be adjusted in the source code for better performance:

- `T`: Prediction horizon (number of steps).
- `delta_t`: Time step for discretization.
- `Q`, `Qf`: State penalty matrices.
- `Rm`: Control penalty matrix.
- `u_min`, `u_max`: Bounds on control inputs.

## Limitations

- Assumes a unicycle robot model.
- Requires a valid reference path for operation.
- May need tuning for specific robot dynamics or environments. 