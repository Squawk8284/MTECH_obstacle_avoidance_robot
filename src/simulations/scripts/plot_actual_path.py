#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
import tf2_ros

class ActualPathPublisher:
    def __init__(self):
        self.path_pub = rospy.Publisher("/actual_path", Path, queue_size=10)
        self.marker_pub = rospy.Publisher("/latest_path_marker", Marker, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.path = Path()
        self.path.header.frame_id = "odom"
        self.path.poses = []

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

        # --- Publish marker at latest point ---
        self.publish_latest_marker(ps)

        # --- Broadcast map -> odom transform ---
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

    def publish_latest_marker(self, pose_stamped):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "actual_path_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully visible

        marker.lifetime = rospy.Duration()  # Keep until overwritten
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node("actual_path_publisher", anonymous=True)
    node = ActualPathPublisher()
    rospy.spin()
