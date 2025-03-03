#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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

def publish_path(path_points):
    """Publish the Bézier curve points to a ROS topic as a Path message."""
    rospy.init_node('bezier_path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/path_topic', Path, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for point in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            # Orientation is left as default (0,0,0,0); if needed you can set a valid quaternion
            pose.pose.orientation.w = 1.0  
            path_msg.poses.append(pose)
        
        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    # Given control points
    control_points = np.array([[0.6, 0.6],
                               [4.37, 0.667],
                               [4.49, 1.226],
                               [6.9, 4.2]])
    
    # Generate Bézier curve path
    path_points = bezier_curve(control_points, num_points=100)
    
    # Publish the path to ROS topic
    try:
        publish_path(path_points)
    except rospy.ROSInterruptException:
        pass
