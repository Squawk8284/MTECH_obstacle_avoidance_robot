#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros

class ActualPathPublisher:
    def __init__(self):
        self.path_pub = rospy.Publisher("/actual_path", Path, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.path = Path()
        self.path.header.frame_id = "odom"
        self.path.poses = []

        # TF broadcaster setup
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def odom_callback(self, msg):
        # --- Record actual path ---
        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = "odom"
        ps.pose = msg.pose.pose

        self.path.poses.append(ps)
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

        # --- Broadcast map -> odom transform ---
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"

        # Static identity transform: replace with actual transform if available
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node("actual_path_publisher", anonymous=True)
    node = ActualPathPublisher()
    rospy.spin()
