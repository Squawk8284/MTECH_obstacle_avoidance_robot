# Obstacle Avoidance Robot Simulation

This repository contains the implementation of an obstacle avoidance robot simulation using ROS (Robot Operating System). The project includes path planning, actual path visualization, and data recording for analysis.

## Project Structure

### 1. **Navigation.py**
- **Path**: `/src/simulations/scripts/Navigation.py`
- **Description**: Implements the `Teaching-Learning-Based Optimization (TLBO)` algorithm for path planning. The script calculates an optimal path from a start to an end point while avoiding obstacles.
- **Key Features**:
    - Bernstein polynomial-based path generation.
    - Collision detection using `Shapely` geometry.
    - Dynamic obstacle extraction from occupancy grids.
    - ROS integration for publishing paths and markers.

### 2. **plot_actual_path.py**
- **Path**: `/src/simulations/scripts/plot_actual_path.py`
- **Description**: Publishes the actual path traversed by the robot based on odometry data. It also visualizes the latest position of the robot using markers.
- **Key Features**:
    - Publishes the actual path as a `Path` message.
    - Visualizes the latest robot position using `Marker`.
    - Broadcasts the `map -> odom` transform.

### 3. **Plots.py**
- **Path**: `/src/simulations/scripts/Plots.py`
- **Description**: Records robot data (e.g., velocity commands, odometry, and goal positions) during simulation. The data is saved as a CSV file for post-simulation analysis.
- **Key Features**:
    - Records `cmd_vel`, `odom`, and goal positions.
    - Automatically stops recording when the robot reaches the goal.
    - Saves data in a timestamped CSV file for analysis.

## How to Run

### Prerequisites
- ROS (Robot Operating System) installed.
- Python 3 with the following libraries:
    - `numpy`
    - `shapely`
    - `pandas`
    - `rospkg`

### Steps
1. **Launch the ROS Environment**:
     - Ensure the ROS master is running.
     - Set up the required ROS parameters (e.g., start and end positions, bounds, topics).

2. **Run the Scripts**:
     - Start the path planning node:
         ```bash
         rosrun simulations Navigation.py
         ```
     - Start the actual path visualization node:
         ```bash
         rosrun simulations plot_actual_path.py
         ```
     - Start the data recording node:
         ```bash
         rosrun simulations Plots.py
         ```

3. **Visualize in RViz**:
     - Add the `Path` and `Marker` topics to visualize the planned and actual paths.

4. **Analyze Data**:
     - After the simulation, check the `plots` directory for the recorded CSV files.

## Parameters

### Navigation.py
- `start_x`, `start_y`: Starting coordinates of the robot.
- `end_x`, `end_y`: Goal coordinates.
- `bounds`: Bounding box for the environment.
- `num_of_learners`: Number of learners for the TLBO algorithm.
- `path_points`: Number of points in the generated path.

### plot_actual_path.py
- `actual_path_topic`: Topic to publish the actual path.
- `latest_path_marker_topic`: Topic to publish the latest position marker.
- `odom_topic`: Topic to subscribe to odometry data.

### Plots.py
- `odom_topic`: Topic for odometry data.
- `cmd_vel_topic`: Topic for velocity commands.
- `path_topic`: Topic for the planned path.
- `goal_tolerance`: Distance threshold to consider the goal as reached.

## Outputs
- **Planned Path**: Published as a `Path` message.
- **Actual Path**: Published as a `Path` message and visualized in RViz.
- **Data Logs**: CSV files containing time-stamped robot data for analysis.

## Directory Structure
```
src/simulations/
├── scripts/
│   ├── Navigation.py
│   ├── plot_actual_path.py
│   ├── Plots.py
├── plots/
│   └── (CSV files generated during simulation)
└── ReadMe.md
```

## License
This project is licensed under the MIT License.

## Acknowledgments
- ROS community for providing tools and libraries.
- Python libraries like `numpy`, `shapely`, and `pandas` for computational support.
