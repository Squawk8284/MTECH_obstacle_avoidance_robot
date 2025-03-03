#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class ActualPathPublisher:
    def __init__(self):
        self.path_pub = rospy.Publisher("/actual_path", Path, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.path = Path()
        self.path.header.frame_id = "map"  # Ensure this matches your TF fixed frame

    def odom_callback(self, msg):
        ps = PoseStamped()
        # Use the odometry header time or current time
        ps.header.stamp = msg.header.stamp  
        ps.header.frame_id = "map"
        ps.pose = msg.pose.pose

        self.path.poses.append(ps)
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node("actual_path_publisher", anonymous=True)
    node = ActualPathPublisher()
    rospy.spin()
