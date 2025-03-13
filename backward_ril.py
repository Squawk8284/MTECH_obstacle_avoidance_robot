#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
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
    Assumes a rotation only around the Z axis.
    Assumes a rotation only around the Z axis.
    """
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)

def publish_path_and_markers(path_points, reverse_path=False):
def publish_path_and_markers(path_points, reverse_path=False):
    """
    Publish the Bézier curve points as a Path message and
    publish visualization markers for the start and end points.
    Publish the Bézier curve points as a Path message and
    publish visualization markers for the start and end points.
    """
    rospy.init_node('bezier_path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/path_topic', Path, queue_size=10)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    if reverse_path:
        path_points = path_points[::-1]
    
    # Define start and end points (for markers)
    start_point = path_points[0]
    end_point = path_points[-1]
    
    if reverse_path:
        path_points = path_points[::-1]
    
    # Define start and end points (for markers)
    start_point = path_points[0]
    end_point = path_points[-1]
    
    while not rospy.is_shutdown():
        # Publish Path message
        # Publish Path message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"
        path_msg.header.frame_id = "odom"
        
        for i, point in enumerate(path_points):
            # Compute derivative for orientation (tangent) estimation
            # Compute derivative for orientation (tangent) estimation
            if i == 0:
                dx = path_points[i+1, 0] - point[0]
                dy = path_points[i+1, 1] - point[1]
            elif i == len(path_points) - 1:
                dx = point[0] - path_points[i-1, 0]
                dy = point[1] - path_points[i-1, 1]
            else:
                dx = path_points[i+1, 0] - path_points[i-1, 0]
                dy = path_points[i+1, 1] - path_points[i-1, 1]
            
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
        
        # Publish start marker
        start_marker = Marker()
        start_marker.header.frame_id = "odom"
        start_marker.header.stamp = rospy.Time.now()
        start_marker.ns = "path_markers"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = start_point[0]
        start_marker.pose.position.y = start_point[1]
        start_marker.pose.position.z = 0.1  # slightly above ground
        start_marker.pose.orientation.x = 0.0
        start_marker.pose.orientation.y = 0.0
        start_marker.pose.orientation.z = 0.0
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.3
        start_marker.scale.y = 0.3
        start_marker.scale.z = 0.3
        start_marker.color.a = 1.0
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        
        marker_pub.publish(start_marker)
        
        # Publish end marker
        end_marker = Marker()
        end_marker.header.frame_id = "odom"
        end_marker.header.stamp = rospy.Time.now()
        end_marker.ns = "path_markers"
        end_marker.id = 1
        end_marker.type = Marker.SPHERE
        end_marker.action = Marker.ADD
        end_marker.pose.position.x = end_point[0]
        end_marker.pose.position.y = end_point[1]
        end_marker.pose.position.z = 0.1
        end_marker.pose.orientation.x = 0.0
        end_marker.pose.orientation.y = 0.0
        end_marker.pose.orientation.z = 0.0
        end_marker.pose.orientation.w = 1.0
        end_marker.scale.x = 0.3
        end_marker.scale.y = 0.3
        end_marker.scale.z = 0.3
        end_marker.color.a = 1.0
        end_marker.color.r = 1.0
        end_marker.color.g = 0.0
        end_marker.color.b = 0.0
        
        marker_pub.publish(end_marker)
        
        
        # Publish start marker
        start_marker = Marker()
        start_marker.header.frame_id = "odom"
        start_marker.header.stamp = rospy.Time.now()
        start_marker.ns = "path_markers"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = start_point[0]
        start_marker.pose.position.y = start_point[1]
        start_marker.pose.position.z = 0.1  # slightly above ground
        start_marker.pose.orientation.x = 0.0
        start_marker.pose.orientation.y = 0.0
        start_marker.pose.orientation.z = 0.0
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.3
        start_marker.scale.y = 0.3
        start_marker.scale.z = 0.3
        start_marker.color.a = 1.0
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        
        marker_pub.publish(start_marker)
        
        # Publish end marker
        end_marker = Marker()
        end_marker.header.frame_id = "odom"
        end_marker.header.stamp = rospy.Time.now()
        end_marker.ns = "path_markers"
        end_marker.id = 1
        end_marker.type = Marker.SPHERE
        end_marker.action = Marker.ADD
        end_marker.pose.position.x = end_point[0]
        end_marker.pose.position.y = end_point[1]
        end_marker.pose.position.z = 0.1
        end_marker.pose.orientation.x = 0.0
        end_marker.pose.orientation.y = 0.0
        end_marker.pose.orientation.z = 0.0
        end_marker.pose.orientation.w = 1.0
        end_marker.scale.x = 0.3
        end_marker.scale.y = 0.3
        end_marker.scale.z = 0.3
        end_marker.color.a = 1.0
        end_marker.color.r = 1.0
        end_marker.color.g = 0.0
        end_marker.color.b = 0.0
        
        marker_pub.publish(end_marker)
        
        rate.sleep()

if __name__ == '__main__':
    # Define control points for your Bézier curve.
    # Define control points for your Bézier curve.
    control_points = np.array([[6.9, 4.2],
                               [4.46, 0.7],
                               [3.65, 0.74],
                               [0.6, 0.6]])
    
    # Generate Bézier curve path.
    path_points = bezier_curve(control_points, num_points=100)
    
    # Set reverse_path True or False depending on desired travel direction.
    # Set reverse_path True or False depending on desired travel direction.
    try:
        publish_path_and_markers(path_points, reverse_path=True)
        publish_path_and_markers(path_points, reverse_path=True)
    except rospy.ROSInterruptException:
        pass
