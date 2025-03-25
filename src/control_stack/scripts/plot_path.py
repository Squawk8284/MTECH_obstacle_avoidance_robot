#!/usr/bin/env python
import rospy
import os
import matplotlib
matplotlib.use('Agg')  # No GUI required
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
from shapely.geometry import Polygon  # Import Shapely for geometric operations

# -------------------------------
# Parameters and Configurations
# -------------------------------

# Define environment bounds as list of (x, y) coordinates (in mm)
bounds = [(0, 0), (0, 4800), (7500, 4800), (7500, 0)]

# Define obstacles as polygons (list of (x, y) coordinates in mm)
obstacles = [
    [(970, 1510), (970, 3290), (3360, 3290), (3360, 1510)],  # Obstacle 1
    [(4140, 3290), (4140, 4800), (5340, 4800), (5340, 3290)]  # Obstacle 2
]

start = [6900, 4200]
end = [600, 600]

# Define safety margin (in mm)
safety_margin = 580  # Safe distance from obstacles and boundary

# Global lists to store received (x, y) coordinates in mm
path_x = []
path_y = []
start_pose = None
end_pose = None
cpp_node_active = True  # Flag to monitor C++ node status

# -------------------------------
# ROS Callback Functions
# -------------------------------

def odom_callback(msg):
    """Callback function for /odom topic to store robot's path in mm."""
    global path_x, path_y, start_pose, end_pose
    x_mm = msg.pose.pose.position.x * 1000.0  # Convert meters to mm
    y_mm = msg.pose.pose.position.y * 1000.0  # Convert meters to mm
    theta = msg.pose.pose.orientation.z  # Approximate heading using quaternion z

    # Store start and end poses
    if len(path_x) == 0:
        start_pose = (x_mm, y_mm, theta)
    end_pose = (x_mm, y_mm, theta)

    path_x.append(x_mm)
    path_y.append(y_mm)
    
    rospy.loginfo("Pose: x=%.1f mm, y=%.1f mm, theta=%.2f rad", x_mm, y_mm, theta)

def check_cpp_node():
    """Checks if the C++ control node is still running."""
    global cpp_node_active
    try:
        cpp_node_active = bool(rospy.get_published_topics())  # If ROS master is running
    except Exception:
        cpp_node_active = False

# -------------------------------
# Plotting Function Using Shapely
# -------------------------------

def plot_path():
    """Plots the robot's path along with the environment bounds (inner boundary)
       and obstacles (outer boundaries) using Shapely."""
    plt.figure(figsize=(10, 10))

    # --- Environment Bounds ---
    # Create Shapely polygon for the environment bounds.
    env_polygon = Polygon(bounds)
    # Compute the inner safe boundary by applying a negative buffer.
    inner_env_polygon = env_polygon.buffer(-safety_margin)
    
    # Plot original environment bounds (closed polygon)
    x_env, y_env = env_polygon.exterior.xy
    plt.plot(x_env, y_env, 'k-', linewidth=2, label="Environment Bounds")
    
    # Plot inner safe boundary if it exists (it may be empty if margin is too large)
    if not inner_env_polygon.is_empty:
        x_inner, y_inner = inner_env_polygon.exterior.xy
        plt.plot(x_inner, y_inner, 'k--', linewidth=2, label="Inner Environment Boundary")

    # --- Obstacles ---
    for idx, obs in enumerate(obstacles):
        # Create Shapely polygon for the obstacle.
        obs_polygon = Polygon(obs)
        # Compute the outer safety boundary (expanded obstacle) by buffering.
        outer_obs_polygon = obs_polygon.buffer(safety_margin)
        
        # Plot the original obstacle (filled polygon)
        x_obs, y_obs = obs_polygon.exterior.xy
        plt.fill(x_obs, y_obs, 'r', alpha=0.5, 
                 label="Obstacle" if idx == 0 else None)
        
        # Plot the expanded (safety) boundary of the obstacle.
        x_outer, y_outer = outer_obs_polygon.exterior.xy
        plt.plot(x_outer, y_outer, 'r--', linewidth=2, 
                 label="Obstacle Safety Boundary" if idx == 0 else None)

    # --- Robot Path ---
    plt.plot(path_x, path_y, 'b-', linewidth=2, marker='o', markersize=5, label="Robot Path")

    # Plot the start and end points
    if start_pose:
        plt.scatter(start_pose[0], start_pose[1], c='g', s=100, label="Start Point")
        plt.arrow(start_pose[0], start_pose[1],
                  np.cos(start_pose[2]) * 200, np.sin(start_pose[2]) * 200,
                  head_width=100, head_length=150, fc='g', ec='g')
    if end_pose:
        plt.scatter(end_pose[0], end_pose[1], c='m', s=100, label="End Point")
        plt.arrow(end_pose[0], end_pose[1],
                  np.cos(end_pose[2]) * 200, np.sin(end_pose[2]) * 200,
                  head_width=100, head_length=150, fc='m', ec='m')

    # Optionally, also plot the raw start and end positions from the defined lists.
    plt.plot(start[0], start[1], 'go', markersize=8, label="Defined Start")
    plt.plot(end[0], end[1], 'mo', markersize=8, label="Defined End")

    # Set labels, title, legend, and grid
    plt.xlabel("X Position (mm)")
    plt.ylabel("Y Position (mm)")
    plt.title("Robot Path with Shapely-based Safety Boundaries")
    plt.legend()
    plt.grid(True)

    # Save the plot in the specified directory
    package_path = os.popen("rospack find control_stack").read().strip()
    path_folder = os.path.join(package_path, "images/path_folder")
    if not os.path.exists(path_folder):
        os.makedirs(path_folder)

    save_path = os.path.join(path_folder, "robot_path.png")
    plt.savefig(save_path, dpi=300)
    rospy.loginfo(f"Plot saved in: {save_path}")

# -------------------------------
# Main Execution Loop
# -------------------------------

def main():
    rospy.init_node('plot_path_node', anonymous=True)

    # Subscribe to /odom topic
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    rospy.loginfo("plot_path_node is running. Waiting for control_node to finish...")

    # Check every 1 second if the C++ node has finished execution
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown() and cpp_node_active:
        check_cpp_node()
        rate.sleep()

    rospy.loginfo("Control node has stopped. Shutting down plotting node...")

    # Generate the plot before exiting
    plot_path()

    # Stop the Python node
    rospy.signal_shutdown("Plotting complete")

if __name__ == '__main__':
    main()
