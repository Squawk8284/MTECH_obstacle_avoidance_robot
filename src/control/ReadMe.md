# Control Package

The `control` package provides the low-level control stack for the NEX robot. It handles motor control, power management, and communication with the robot's hardware components via serial communication.

## Features

- **Motor Control**: Supports open-loop, closed-loop, and position control modes.
- **Power Management**: Provides APIs to monitor battery voltage, current, and temperature.
- **Inertial Control**: Interfaces with IMU sensors for orientation and motion data.
- **ROS Integration**: Subscribes to `/cmd_vel` for velocity commands and publishes odometry data.

## Dependencies

The `control` package depends on the following ROS and system packages:

- `roscpp`
- `std_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf`
- `serial`

## Build Instructions

To build the workspace:

```bash
cd ~/catkin_ws
catkin_make
```
### Code Structure

The `control` package is organized as follows:

- **Source Files**:
    - `control.cpp`: Main entry point for the control node. Initializes ROS, sets up parameters, and handles the control loop.

- **Header Files**:
    - `nex_robot.hpp`: Core utilities for serial communication and command execution.
    - `motor_control_api.hpp`: APIs for motor control, including velocity and mode settings.
    - `power_management_api.hpp`: APIs for monitoring battery voltage, current, and temperature.
    - `inertial_control_api.hpp`: APIs for interfacing with IMU sensors.
    - `ros_callbacks.hpp`: ROS-specific callbacks for handling subscribed topics.
    - `user_defined_functions.hpp`: Custom functions for robot initialization, odometry updates, and velocity commands.

### Key Functions

- **Initialization**:
    - `init()`: Configures the robot's safety mode, wheel diameter, axle length, and encoder settings.

- **Velocity Control**:
    - `CmdLinearVelocity_mps(float linearVelocity, float angularVelocity)`: Computes and sets motor velocities based on linear and angular velocity commands.

- **Odometry Updates**:
    - `UpdateOdometry(geometry_msgs::TransformStamped &odom_trans, nav_msgs::Odometry &odom_msg)`: Updates the robot's pose and publishes odometry data.

- **Power Monitoring**:
    - `ReadBatteryVoltage()`, `ReadBatteryCurrent()`, `ReadBatteryTemperature()`: Functions to monitor the robot's battery status.

- **IMU Integration**:
    - `get3AxisAccelorometer()`, `get3AxisGyroscope()`, `get3AxisMagnetometer()`: Functions to retrieve IMU sensor data.

### Notes

- Ensure the serial ports (`/dev/ttyRobot` and `/dev/ttyIMU`) are correctly configured and accessible.
- The robot's hardware parameters (e.g., wheel diameter, axle length) must match the actual hardware specifications.
- Debugging can be enabled by uncommenting the `#define DEBUG` directive in the source files.
- The `control.launch` file should be updated with the correct parameter file path for your setup.
Source the workspace:

```bash
source ~/catkin_ws/devel/setup.bash
```

## Usage

### Launching the Control Node

To start the control node, use the provided launch file:

```bash
roslaunch control control.launch param_file:=<src/simulations/config/config.yaml>
```

### Parameters

The following parameters can be configured via the parameter file:

- `cmd_vel_topic`: Topic for velocity commands (default: `/cmd_vel`).
- `odom_topic`: Topic for odometry data (default: `/odom`).
- `start_x`, `start_y`, `start_theta`: Initial pose of the robot.

### Topics

#### Subscribed Topics:

- `/cmd_vel`: Receives velocity commands for the robot.

#### Published Topics:

- `/odom`: Publishes odometry data.
