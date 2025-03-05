#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

def bernstein_poly(i, n, t):
    """Compute the Bernstein polynomial basis function."""
    from math import comb
    return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

def bezier_curve(control_points, num_points=100):
    """Generate Bézier curve points from control points."""
    n = len(control_points) - 1
    curve = np.zeros((num_points, 2))
    t_values = np.linspace(0, 1, num_points)
    
    for i in range(n + 1):
        curve += np.outer(bernstein_poly(i, n, t_values), control_points[i])
    
    return curve

def quaternion_from_yaw(yaw):
    """
    Convert a yaw angle (in radians) into a quaternion (x, y, z, w).
    This simple conversion assumes a rotation only around the Z axis.
    """
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)

def publish_path(path_points):
    """
    Publish the Bézier curve points to a ROS topic as a Path message.
    Each PoseStamped is given an orientation based on the tangent of the path.
    """
    rospy.init_node('bezier_path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/path_topic', Path, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for i, point in enumerate(path_points):
            # Approximate the derivative (tangent) of the path at the current point.
            if i == 0:
                # Forward difference for the first point
                dx = path_points[i+1, 0] - point[0]
                dy = path_points[i+1, 1] - point[1]
            elif i == len(path_points) - 1:
                # Backward difference for the last point
                dx = point[0] - path_points[i-1, 0]
                dy = point[1] - path_points[i-1, 1]
            else:
                # Central difference for the intermediate points
                dx = path_points[i+1, 0] - path_points[i-1, 0]
                dy = path_points[i+1, 1] - path_points[i-1, 1]
            
            # Compute yaw angle from the derivative
            yaw = math.atan2(dy, dx)
            q = quaternion_from_yaw(yaw)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path_msg.poses.append(pose)
        
        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    # Given control points
    control_points = np.array([[6.9, 4.2],
                               [4.46, 0.7],
                               [3.65, 0.74],
                               [0.6, 0.6]])
    
    # Generate Bézier curve path
    path_points = bezier_curve(control_points, num_points=100)
    
    # Publish the path to the ROS topic
    try:
        publish_path(path_points)
    except rospy.ROSInterruptException:
        pass
