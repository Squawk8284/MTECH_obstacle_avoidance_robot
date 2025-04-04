import os
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdometryPlotter:
    def __init__(self):
        self.x_data = []
        self.y_data = []
        self.z_data = []  # Time (or steps) since motion started

        self.active = False
        self.start_time = None

        # Set up 3D plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Set save path: simulations/plots
        package_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
        self.save_dir = os.path.join(package_path, 'plots')
        os.makedirs(self.save_dir, exist_ok=True)

        # ROS Subscribers
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # Register shutdown hook
        rospy.on_shutdown(self.save_plot)

    def cmd_vel_callback(self, msg):
        if not self.active:
            self.active = True
            self.start_time = rospy.Time.now()
            rospy.loginfo("Motion started — recording odometry data.")

    def odom_callback(self, msg):
        if not self.active:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = (msg.header.stamp - self.start_time).to_sec()

        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(t)

    def save_plot(self):
        if not self.active or not self.x_data:
            rospy.logwarn("No motion data recorded. Plot not saved.")
            return

        self.ax.clear()

        # Plot odometry path only
        self.ax.plot3D(self.x_data, self.y_data, self.z_data,
                       label="Odometry Path", color='blue')

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Time (s)")
        self.ax.set_title("3D Odometry Path")
        self.ax.legend()

        filepath = os.path.join(self.save_dir, "odom_3d_plot.png")
        self.fig.savefig(filepath)
        rospy.loginfo(f"3D odometry plot saved at: {filepath}")

if __name__ == "__main__":
    rospy.init_node("odom_plotter")
    OdometryPlotter()
    rospy.spin()
