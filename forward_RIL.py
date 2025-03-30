#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import math
from math import comb  # Ensure compatibility with Python 3.8+

def bernstein_poly(i, n, t):
    """Compute the Bernstein polynomial basis function."""
    return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

def bezier_curve(control_points, num_points=25):
    """Generate Bézier curve points from control points."""
    n = len(control_points) - 1
    curve = np.zeros((num_points, 2))
    t_values = np.linspace(0, 1, num_points)
    
    for i in range(n + 1):
        curve += np.outer(bernstein_poly(i, n, t_values), control_points[i])
    
    return curve

def quaternion_from_yaw(yaw):
    """Convert a yaw angle (in radians) into a quaternion (x, y, z, w)."""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)

def publish_path_and_markers(path_points, reverse_path=False):
    """Publish the Bézier curve points as a Path message and visualization markers."""
    rospy.init_node('bezier_path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/path_topic', Path, queue_size=10)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    if reverse_path:
        path_points = path_points[::-1]
    
    # Define start and end points for markers
    start_point, end_point = path_points[0], path_points[-1]

    while not rospy.is_shutdown():
        # Publish Path message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"
        
        for i, point in enumerate(path_points):
            # Compute derivative for orientation (tangent) estimation
            if i == 0:
                dx, dy = path_points[i+1] - point
            elif i == len(path_points) - 1:
                dx, dy = point - path_points[i-1]
            else:
                dx, dy = path_points[i+1] - path_points[i-1]
            
            yaw = math.atan2(dy, dx)
            q = quaternion_from_yaw(yaw)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x, pose.pose.position.y = point
            pose.pose.position.z = 0.0
            pose.pose.orientation.x, pose.pose.orientation.y = q[:2]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2:]

            path_msg.poses.append(pose)
        
        path_pub.publish(path_msg)

        # Publish markers for start and end points
        for i, (p, color) in enumerate([(start_point, (0, 1, 0)), (end_point, (1, 0, 0))]):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "path_markers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = p
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.a, marker.color.r, marker.color.g, marker.color.b = (1.0, *color)
            
            marker_pub.publish(marker)

        rate.sleep()

if __name__ == '__main__':
    # Define control points for Bézier curve.
    control_points = np.array([
        [5,1],
        [6.194,3.008],
        [6.309,3.203],
        [6.9,4.2]
    ])
    
    # Generate Bézier curve path.
    path_points = bezier_curve(control_points, num_points=100)

    try:
        publish_path_and_markers(path_points, reverse_path=False)
    except rospy.ROSInterruptException:
        pass
