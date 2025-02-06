#!/usr/bin/env python

import rospy
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np

# ------------------ Parameters ------------------
bounds = [(0, 0), (0, 4800), (7530, 4800), (7530,0)]  
obstacles = [
    [(4170,3300),(4170,4800),(5370,4800),(5370,3300)],  
    [(610,1670),(610,3450),(3390,3450),(3390,1670)]  
]
safety_margin = 300  

path_x, path_y = [], []
start_pose, end_pose = None, None
shutdown_received = False  

# ------------------ Callback Functions ------------------

def odom_callback(msg):
    """Stores robot's position from /odom topic."""
    global path_x, path_y, start_pose, end_pose
    x = msg.pose.pose.position.x * 1000
    y = msg.pose.pose.position.y * 1000
    theta = msg.pose.pose.orientation.z

    if len(path_x) == 0:
        start_pose = (x, y, theta)
    end_pose = (x, y, theta)

    path_x.append(x)
    path_y.append(y)

def shutdown_callback(msg):
    """Handles shutdown signal from the C++ node."""
    global shutdown_received
    shutdown_received = True
    rospy.signal_shutdown("Received shutdown signal")

# ------------------ Plotting Function ------------------

def plot_path():
    """Generates the robot's trajectory plot."""
    plt.figure(figsize=(8, 8))

    # Plot environment bounds
    bounds_x, bounds_y = zip(*bounds + [bounds[0]])  
    plt.plot(bounds_x, bounds_y, 'k-', linewidth=2, label="Bounds")

    # Plot obstacles
    for obs in obstacles:
        obs_x, obs_y = zip(*obs + [obs[0]])
        plt.fill(obs_x, obs_y, 'r', alpha=0.5, label="Obstacle")

    plt.plot(path_x, path_y, 'b-', linewidth=2, marker='o', label="Path")

    # Get the path to the ROS package
    package_path = os.popen("rospack find control_stack").read().strip()
    
    # Define the path_folder inside the package
    path_folder = os.path.join(package_path, "images/path_folder")
    
    # Ensure the directory exists
    if not os.path.exists(path_folder):
        os.makedirs(path_folder)
    
    save_path = os.path.join(path_folder, "robot_path.png")

    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.title("Robot Path with Obstacles")
    plt.legend()
    plt.savefig(save_path, dpi=300)
    rospy.loginfo("Plot saved.")

# ------------------ Main ------------------

def main():
    rospy.init_node('plot_path_node')
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/shutdown_signal", Bool, shutdown_callback)
    rospy.spin()
    plot_path()
    return

if __name__ == '__main__':
    main()
