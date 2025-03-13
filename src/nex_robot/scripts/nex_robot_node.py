#!/usr/bin/env python
import rospy
import serial
import struct
import math
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import quaternion_from_euler

def calculate_checksum(packet_bytes):
    s = sum(packet_bytes) & 0xFF
    checksum = ((~s) + 1) & 0xFF
    return checksum

class NexRobotNode(object):
    def __init__(self):
        rospy.init_node('nex_robot_node')
        
        # Load ports and parameters from configuration.
        port_robot = rospy.get_param("ports/robot", "/ttyRobot")
        port_imu   = rospy.get_param("ports/imu", "/ttyIMU")
        baud = rospy.get_param("~baud", 57600)
        
        # Open two serial connections: one for robot and one for the IMU.
        self.robot_serial = serial.Serial(port_robot, baud, timeout=0.1)
        self.imu_serial   = serial.Serial(port_imu, baud, timeout=0.1)
        
        # Robot parameters.
        self.wheel_radius = rospy.get_param('robot/wheel_radius', 0.033)
        self.wheel_base = rospy.get_param('robot/wheel_base', 0.15)
        self.encoder_resolution = rospy.get_param('robot/encoder_resolution', 1024)
        
        # Topic configuration.
        self.cmd_vel_topic = rospy.get_param('topics/cmd_vel', '/cmd_vel')
        self.odom_topic = rospy.get_param('topics/odom', '/odom')
        
        # Initialize pose and previous encoder values.
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.left_encoder_prev = 0
        self.right_encoder_prev = 0
        self.last_time = rospy.Time.now()
        
        # Commanded velocities from /cmd_vel.
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        
        # Publishers and subscribers.
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=50)
        self.imu_pub  = rospy.Publisher("imu", Imu, queue_size=50)
        self.cmd_vel_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
    def cmd_vel_callback(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

    def send_robot_command(self, command, data=[]):
        # Build packet for robot serial port.
        packet = bytearray(b'NEX')
        packet.append(command)
        for d in data:
            packet.append(d)
        checksum = calculate_checksum(packet)
        packet.append(checksum)
        self.robot_serial.write(packet)

    def send_imu_command(self, command, data=[]):
        # Build packet for IMU serial port.
        packet = bytearray(b'NEX')
        packet.append(command)
        for d in data:
            packet.append(d)
        checksum = calculate_checksum(packet)
        packet.append(checksum)
        self.imu_serial.write(packet)

    def read_robot_response(self, expected_length):
        return self.robot_serial.read(expected_length)

    def read_imu_response(self, expected_length):
        return self.imu_serial.read(expected_length)

    def get_encoder_counts(self):
        # Using placeholder commands (0x93 for left and 0x94 for right)
        self.send_robot_command(0x93)
        resp_left = self.read_robot_response(7)  # adjust length per manual
        if len(resp_left) == 7 and chr(resp_left[0]) in ['S', 'F']:
            left_count = struct.unpack(">i", resp_left[3:7])[0]
        else:
            left_count = self.left_encoder_prev

        self.send_robot_command(0x94)
        resp_right = self.read_robot_response(7)
        if len(resp_right) == 7 and chr(resp_right[0]) in ['S', 'F']:
            right_count = struct.unpack(">i", resp_right[3:7])[0]
        else:
            right_count = self.right_encoder_prev

        return left_count, right_count

    def get_imu_data(self):
        # Use a placeholder command (e.g., 0x42) for reading IMU data.
        self.send_imu_command(0x42)
        resp = self.read_imu_response(11)  # adjust expected length per manual
        if len(resp) == 11 and chr(resp[0]) in ['S', 'F']:
            # Assume 2 bytes per axis for gyroscope data (big-endian signed short).
            gx = struct.unpack(">h", resp[2:4])[0] / 1000.0
            gy = struct.unpack(">h", resp[4:6])[0] / 1000.0
            gz = struct.unpack(">h", resp[6:8])[0] / 1000.0
        else:
            gx, gy, gz = 0.0, 0.0, 0.0
        return gx, gy, gz

    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        left_count, right_count = self.get_encoder_counts()
        d_left = left_count - self.left_encoder_prev
        d_right = right_count - self.right_encoder_prev
        self.left_encoder_prev = left_count
        self.right_encoder_prev = right_count

        # Convert encoder counts to distances.
        dist_left = (2 * math.pi * self.wheel_radius * d_left) / float(self.encoder_resolution)
        dist_right = (2 * math.pi * self.wheel_radius * d_right) / float(self.encoder_resolution)
        d = (dist_left + dist_right) / 2.0
        dth = (dist_right - dist_left) / self.wheel_base

        # Update pose.
        self.x += d * math.cos(self.th + dth / 2.0)
        self.y += d * math.sin(self.th + dth / 2.0)
        self.th += dth

        # Broadcast transform.
        quat = quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quat,
            current_time,
            "base_link",
            "odom"
        )

        # Publish odometry.
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = d / dt if dt > 0 else 0.0

        # For angular velocity, we can use the IMU z-axis value.
        _, _, gz = self.get_imu_data()
        odom.twist.twist.angular.z = gz
        self.odom_pub.publish(odom)

        self.last_time = current_time

    def send_velocity_command(self, linear, angular):
        # Differential drive conversion.
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)

        left_mm = int(v_left * 1000)
        right_mm = int(v_right * 1000)
        left_bytes = [(left_mm >> 8) & 0xFF, left_mm & 0xFF]
        right_bytes = [(right_mm >> 8) & 0xFF, right_mm & 0xFF]

        # Send velocity commands using placeholder command codes:
        # 0x70 for left motor and 0x71 for right motor.
        self.send_robot_command(0x70, [0x01] + left_bytes)
        self.send_robot_command(0x71, [0x01] + right_bytes)

    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz loop rate.
        while not rospy.is_shutdown():
            self.update_odometry()
            # Use the latest /cmd_vel values to set velocity.
            self.send_velocity_command(self.cmd_linear, self.cmd_angular)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = NexRobotNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
