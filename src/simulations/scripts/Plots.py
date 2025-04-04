#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import pandas as pd
from datetime import datetime
import threading
import os
import rospkg
import math

class RecorderNode:
    def __init__(self, goal_tolerance=0.2):
        rospy.init_node('topic_recorder', anonymous=True)
        self.goal_tolerance = goal_tolerance

        self.latest = {
            'cmd_vel': None,
            'odom': None,
        }

        # Path info (not for CSV)
        self.current_goal_position = None

        self.records = []
        self.lock = threading.Lock()
        self.recording_enabled = False

        rospy.Subscriber('/cmd_vel', Twist, self.cb_cmd_vel, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.cb_odom, queue_size=1)
        rospy.Subscriber('/path_topic', Path, self.cb_path, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("RecorderNode started, waiting for /cmd_vel to begin recording...")
        rospy.spin()

    def cb_cmd_vel(self, msg: Twist):
        with self.lock:
            self.latest['cmd_vel'] = msg
            if not self.recording_enabled:
                rospy.loginfo("First /cmd_vel received, starting recording.")
            self.recording_enabled = True
            self._maybe_record()

    def cb_odom(self, msg: Odometry):
        with self.lock:
            self.latest['odom'] = msg
            if self.recording_enabled:
                self._maybe_record()
                self._check_goal_and_shutdown()

    def cb_path(self, msg: Path):
        """Update only the latest goal position (last pose)"""
        if msg.poses:
            last = msg.poses[-1].pose.position
            self.current_goal_position = (last.x, last.y)

    def _maybe_record(self):
        if not self.recording_enabled:
            return

        ts = rospy.Time.now().to_sec()
        lv = self.latest['cmd_vel']
        od = self.latest['odom']

        row = {
            'time': ts,
            'cmd_lin_x': lv.linear.x,
            'cmd_lin_y': lv.linear.y,
            'cmd_lin_z': lv.linear.z,
            'cmd_ang_x': lv.angular.x,
            'cmd_ang_y': lv.angular.y,
            'cmd_ang_z': lv.angular.z,
            'odom_pos_x': od.pose.pose.position.x if od else None,
            'odom_pos_y': od.pose.pose.position.y if od else None,
            'odom_pos_z': od.pose.pose.position.z if od else None,
            'odom_ori_x': od.pose.pose.orientation.x if od else None,
            'odom_ori_y': od.pose.pose.orientation.y if od else None,
            'odom_ori_z': od.pose.pose.orientation.z if od else None,
            'odom_ori_w': od.pose.pose.orientation.w if od else None,
        }
        self.records.append(row)

    def _check_goal_and_shutdown(self):
        if self.current_goal_position and self.latest['odom']:
            goal_x, goal_y = self.current_goal_position
            odom = self.latest['odom'].pose.pose.position
            dx = odom.x - goal_x
            dy = odom.y - goal_y
            dist = math.hypot(dx, dy)
            if dist <= self.goal_tolerance:
                rospy.loginfo(f"Goal reached (distance {dist:.3f} ≤ {self.goal_tolerance}). Saving and shutting down.")
                rospy.signal_shutdown('Goal reached')

    def on_shutdown(self):
        """Save CSV on shutdown if any data is recorded"""
        if not self.recording_enabled or not self.records:
            rospy.loginfo("No data recorded. Exiting without saving.")
            return

        df = pd.DataFrame(self.records)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('simulations')
        base_dir = os.path.join(pkg_path, 'plots')
        os.makedirs(base_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = os.path.join(base_dir, f"record_{timestamp}")
        os.makedirs(session_dir, exist_ok=True)

        csv_name = f"record_{timestamp}.csv"
        csv_path = os.path.join(session_dir, csv_name)

        try:
            df.to_csv(csv_path, index=False)
            rospy.loginfo(f"Recorded {len(df)} samples. Saved to {csv_path}")
        except Exception as e:
            rospy.logerr(f"Error saving CSV: {e}")


if __name__ == '__main__':
    try:
        RecorderNode(goal_tolerance=0.2)
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received. Shutting down and saving data...")
