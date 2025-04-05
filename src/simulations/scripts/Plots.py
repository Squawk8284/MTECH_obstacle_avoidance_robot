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
    def __init__(self, goal_tolerance=0.1):
        rospy.init_node('topic_recorder', anonymous=True)
        self.goal_tolerance = goal_tolerance

        # Storage for the latest message of each type
        self.latest = {
            'cmd_vel': None,
            'odom': None,
            'path_topic': None,
        }

        # Flattened path as string for saving to CSV
        self.flattened_path_str = None

        # Buffer of records
        self.records = []

        # Lock for thread safety
        self.lock = threading.Lock()

        # Flag: have we ever received a cmd_vel?
        self.recording_enabled = False

        # Subscribers
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
                self._check_goal_and_shutdown()
                self._maybe_record()

    def cb_path(self, msg: Path):
        with self.lock:
            self.latest['path_topic'] = msg
            # Flatten the full path to a single string: "x1,y1;x2,y2;..."
            self.flattened_path_str = ";".join([
                f"{pose.pose.position.x:.3f},{pose.pose.position.y:.3f}"
                for pose in msg.poses
            ])

    def _maybe_record(self):
        """Record one sample if recording is enabled."""
        if not self.recording_enabled:
            return

        ts = rospy.Time.now().to_sec()
        lv = self.latest['cmd_vel']
        od = self.latest['odom']

        row = {
            'time': ts,
            # cmd_vel
            'cmd_lin_x': lv.linear.x,
            'cmd_lin_y': lv.linear.y,
            'cmd_lin_z': lv.linear.z,
            'cmd_ang_x': lv.angular.x,
            'cmd_ang_y': lv.angular.y,
            'cmd_ang_z': lv.angular.z,
            # odom pose
            'odom_pos_x': od.pose.pose.position.x if od else None,
            'odom_pos_y': od.pose.pose.position.y if od else None,
            'odom_pos_z': od.pose.pose.position.z if od else None,
            'odom_ori_x': od.pose.pose.orientation.x if od else None,
            'odom_ori_y': od.pose.pose.orientation.y if od else None,
            'odom_ori_z': od.pose.pose.orientation.z if od else None,
            'odom_ori_w': od.pose.pose.orientation.w if od else None,
            # path_topic: number of poses
            'path_topic_count': len(self.latest['path_topic'].poses) if self.latest['path_topic'] else None,
            # NEW: flattened path string
            'path_topic_points': self.flattened_path_str if self.flattened_path_str else '',
        }
        self.records.append(row)

    def _check_goal_and_shutdown(self):
        """If odom is within tolerance of the last path_topic pose, shut down."""
        od = self.latest['odom']
        pt = self.latest['path_topic']
        if od and pt and pt.poses:
            goal = pt.poses[-1].pose.position
            dx = od.pose.pose.position.x - goal.x
            dy = od.pose.pose.position.y - goal.y
            dist = math.hypot(dx, dy)
            if dist <= self.goal_tolerance:
                rospy.loginfo(f"Goal reached (distance {dist:.3f} ≤ {self.goal_tolerance}). Saving and shutting down.")
                rospy.signal_shutdown('goal reached')

    def on_shutdown(self):
        """Called on rospy shutdown; write CSV into its own timestamped subfolder."""
        if not self.recording_enabled or not self.records:
            rospy.loginfo("No data recorded. Exiting without creating files.")
            return

        # Prepare DataFrame
        df = pd.DataFrame(self.records)

        # Resolve package path and ensure base plots/ exists
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('simulations')
        base_plots_dir = os.path.join(pkg_path, 'plots')
        os.makedirs(base_plots_dir, exist_ok=True)

        # Create timestamped subfolder
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = os.path.join(base_plots_dir, f"record_{timestamp}")
        os.makedirs(session_dir, exist_ok=True)

        # Write CSV
        csv_name = f"record_{timestamp}.csv"
        csv_path = os.path.join(session_dir, csv_name)
        try:
            df.to_csv(csv_path, index=False)
            rospy.loginfo(f"Recorded {len(df)} samples. Saved CSV to {csv_path}")
        except Exception as e:
            rospy.logerr(f"Failed to write CSV file: {e}")

if __name__ == '__main__':
    try:
        RecorderNode(goal_tolerance=0.2)
    except rospy.ROSInterruptException:
        pass
