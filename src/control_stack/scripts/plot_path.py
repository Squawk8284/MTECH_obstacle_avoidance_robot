#!/usr/bin/env python

import rospy
import os
import matplotlib
matplotlib.use('Agg')  # No GUI required
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np

# -------------------------------
# Parameters and Configurations
# -------------------------------

# Define environment bounds as list of (x, y) coordinates
bounds = [(0, 0), (0, 4800), (7500, 4800), (7500, 0)]

# Define obstacles as polygons (list of (x, y) coordinates)
obstacles = [
    [(970, 1510), (970, 3290), (3360, 3290), (3360, 1510)],  # Obstacle 1
    [(4140, 3290), (4140, 4800), (5340, 4800), (5340, 3290)]  # Obstacle 2
]

start = [600,600]
end = [6900, 4200]

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
    except:
        cpp_node_active = False

# -------------------------------
# Utility Functions for Safety Margins
# -------------------------------

def shrink_polygon(polygon, margin):
    """Shrinks a polygon by the given margin (used for inner boundary)."""
    return [(x + margin if x > 0 else x - margin, 
             y + margin if y > 0 else y - margin) for x, y in polygon]

def expand_polygon(polygon, margin):
    """Expands a polygon by the given margin (used for obstacle safety)."""
    return [(x - margin if x > 0 else x + margin, 
             y - margin if y > 0 else y + margin) for x, y in polygon]

# -------------------------------
# Plotting Function
# -------------------------------

def plot_path():
    """Plots the robot's path along with bounds, obstacles, and safety margins."""
    plt.figure(figsize=(10, 10))

    # Plot environment bounds
    bound_x, bound_y = zip(*bounds + [bounds[0]])  # Close the shape
    plt.plot(bound_x, bound_y, 'k-', linewidth=2, label="Environment Bounds")

    # Plot safety margin **inside** the bounds (shrinked version)
    safe_bounds = shrink_polygon(bounds, safety_margin)
    safe_bound_x, safe_bound_y = zip(*safe_bounds + [safe_bounds[0]])
    plt.plot(safe_bound_x, safe_bound_y, 'k--', linewidth=1, label="Inner Safety Margin")

    # Plot obstacles and their **external** safety margins (expanded version)
    for obs in obstacles:
        obs_x, obs_y = zip(*obs + [obs[0]])  # Close the shape
        plt.fill(obs_x, obs_y, 'r', alpha=0.5, label="Obstacle" if obs == obstacles[0] else "")

        # Safety margin around the obstacle (expanded)
        safe_obs = expand_polygon(obs, safety_margin)
        safe_obs_x, safe_obs_y = zip(*safe_obs + [safe_obs[0]])
        plt.plot(safe_obs_x, safe_obs_y, 'r--', linewidth=1, label="Obstacle Safety Margin" if obs == obstacles[0] else "")

    # Plot the robot's path
    plt.plot(path_x, path_y, 'b-', linewidth=2, marker='o', markersize=5, label="Robot Path")

    plt.plot(start[0],start[1], 'g+', linewidth=5, marker='+', markersize=5, label="End")
    plt.plot(path_x, path_y, 'g+', linewidth=5, marker='+', markersize=5, label="End")

    # Plot start and end points with orientation arrows
    if start_pose:
        plt.scatter(start_pose[0], start_pose[1], c='g', s=100, label="Start Point")
        plt.arrow(start_pose[0], start_pose[1], np.cos(start_pose[2]) * 200, np.sin(start_pose[2]) * 200, head_width=100, head_length=150, fc='g', ec='g')

    if end_pose:
        plt.scatter(end_pose[0], end_pose[1], c='m', s=100, label="End Point")
        plt.arrow(end_pose[0], end_pose[1], np.cos(end_pose[2]) * 200, np.sin(end_pose[2]) * 200, head_width=100, head_length=150, fc='m', ec='m')

    # Set labels and grid
    plt.xlabel("X Position (mm)")
    plt.ylabel("Y Position (mm)")
    plt.title("Robot Path with Obstacles and Safety Margins")
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
