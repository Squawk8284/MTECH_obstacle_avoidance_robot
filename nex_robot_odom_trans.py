#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import Twist as CmdVel

class GazeboOdomPublisher:
    def __init__(self):
        rospy.init_node('gazebo_odom_publisher', anonymous=True)
        
        self.model_name = "0xDelta"  # Model name to track
        
        # Subscriber for Gazebo model states
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # Subscriber for /cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', CmdVel, self.cmd_vel_callback)
        
        # Publisher for odometry
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Publisher for velocity control
        self.cmd_vel_pub = rospy.Publisher('/delta_velocity_controller/cmd_vel', CmdVel, queue_size=10)

    def model_states_callback(self, msg):
        try:
            # Find index of the model
            model_index = msg.name.index(self.model_name)
            
            # Extract pose and twist
            pose = msg.pose[model_index]
            twist = msg.twist[model_index]
            
            # Create odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            
            odom_msg.pose.pose = pose
            odom_msg.twist.twist = twist
            
            # Publish odometry
            self.pub.publish(odom_msg)
            rospy.loginfo("Published odom for %s", self.model_name)
        except ValueError:
            rospy.logwarn("Model %s not found in /gazebo/model_states", self.model_name)

    def cmd_vel_callback(self, msg):
        # Forward received cmd_vel to the new topic
        self.cmd_vel_pub.publish(msg)
        rospy.loginfo("Forwarded cmd_vel to /delta_velocity_controller/cmd_vel")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GazeboOdomPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
