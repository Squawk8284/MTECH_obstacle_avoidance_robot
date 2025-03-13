#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "std_msgs/Bool.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "0xRobotcpplib.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <serial/serial.h>
#include <sstream>
#include <string>

//----------------------------------------------------------------------
// Constants & Macros (All in SI units)
//----------------------------------------------------------------------

#define DEVICE_PORT "/dev/ttyRobot"      // Robot control serial port

// Default hardware values in meters.
#define DEFAULT_WHEEL_DIAMETER    (0.26)   // 260 mm => 0.26 m
#define DEFAULT_AXLE_LENGTH       (0.59)    // 590 mm => 0.59 m
#define DEFAULT_ENCODER_TICKS     (2000)   // ticks per revolution

// Compute wheel radius (in meters) and distance per encoder tick (in meters)
#define WHEEL_RADIUS  (DEFAULT_WHEEL_DIAMETER / 2.0)
#define DISTANCE_PER_TICK  ((M_PI * DEFAULT_WHEEL_DIAMETER) / (DEFAULT_ENCODER_TICKS))

#define DEG_TO_RAD (M_PI/180.0)
#define ACCEL_CONVERSION (9.806/256.0)
#define GYRO_CONVERSION (M_PI/180.0)

//----------------------------------------------------------------------
// Global Variables for Odometry & Robot Control
//----------------------------------------------------------------------

// Encoder previous counts
int32_t prevLeftEncoder = 0;
int32_t prevRightEncoder = 0;

// Global robot pose (in meters for x, y and radians for theta)
double globalX = 0.0, globalY = 0.0, globalTheta = 0.0;

// Last received velocity command (for odometry twist)
geometry_msgs::Twist g_lastCmdVel;

// Pointers to our robot object and serial handle (set once in main)
lib0xRobotCpp* g_robot = nullptr;
void* g_hSerial = nullptr;

//----------------------------------------------------------------------
// Helper Functions
//----------------------------------------------------------------------

// Update the robot pose using encoder counts.
// The pose (globalX, globalY, globalTheta) is updated in-place.
// All distances are in meters.
void updateRobotPosition(lib0xRobotCpp &robot, void* hSerial,
                           double &x, double &y, double &theta,
                           double distancePerTick, double axleLength) {
    int32_t currentLeftEncoder = 0;
    int32_t currentRightEncoder = 0;
    
    robot.getLeftMotorCount(hSerial, &currentLeftEncoder);
    robot.getRightMotorCount(hSerial, &currentRightEncoder);
    
    ROS_INFO("Encoders: Left=%d, Right=%d", currentLeftEncoder, currentRightEncoder);
    
    int32_t dLeft = currentLeftEncoder - prevLeftEncoder;
    int32_t dRight = currentRightEncoder - prevRightEncoder;
    
    prevLeftEncoder = currentLeftEncoder;
    prevRightEncoder = currentRightEncoder;
    
    double dLeft_m = dLeft * distancePerTick;
    double dRight_m = dRight * distancePerTick;
    
    double dCenter = (dLeft_m + dRight_m) / 2.0;
    double dTheta = (dRight_m - dLeft_m) / axleLength;
    
    double thetaMid = theta + dTheta / 2.0;
    x += dCenter * cos(thetaMid);
    y += dCenter * sin(thetaMid);
    theta += dTheta;
    
    // Normalize theta between -pi and pi.
    while (theta > M_PI)  theta -= 2 * M_PI;
    while (theta < -M_PI) theta += 2 * M_PI;
    
    ROS_INFO("Odometry Pose: x=%.3f m, y=%.3f m, theta=%.2f rad", x, y, theta);
}

// Parse a raw IMU data string received from the Razor IMU.
// The expected format is: "#YPRAG=<yaw>,<pitch>,<roll>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>"
sensor_msgs::Imu parseIMUData(const std::string& data) {
    sensor_msgs::Imu imu_msg;
    if (data.size() < 7)
        return imu_msg;
    std::string payload = data.substr(7); // Remove "#YPRAG="
    std::stringstream ss(payload);
    std::vector<std::string> tokens;
    std::string token;
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }
    if (tokens.size() != 9) {
        ROS_WARN("IMU data does not have 9 tokens");
        return imu_msg;
    }
    // Optionally adjust yaw calibration here
    double imu_yaw_calibration = 0.0;
    double yaw   = -std::stod(tokens[0]) + imu_yaw_calibration;
    double pitch = -std::stod(tokens[1]) * DEG_TO_RAD;
    double roll  =  std::stod(tokens[2]) * DEG_TO_RAD;
    
    tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    
    imu_msg.linear_acceleration.x = std::stod(tokens[3]) * ACCEL_CONVERSION;
    imu_msg.linear_acceleration.y = std::stod(tokens[4]) * ACCEL_CONVERSION;
    imu_msg.linear_acceleration.z = std::stod(tokens[5]) * ACCEL_CONVERSION;
    
    imu_msg.angular_velocity.x = std::stod(tokens[6]) * GYRO_CONVERSION;
    imu_msg.angular_velocity.y = std::stod(tokens[7]) * GYRO_CONVERSION;
    imu_msg.angular_velocity.z = std::stod(tokens[8]) * GYRO_CONVERSION;
    
    // Set covariance values for use with robot_localization.
    // Here we set diagonal elements; adjust as needed.
    for (int i = 0; i < 9; ++i) {
        imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.5 : 0.0;
        imu_msg.angular_velocity_covariance[i]    = (i % 4 == 0) ? 0.001 : 0.0;
        imu_msg.orientation_covariance[i]           = (i % 4 == 0) ? 0.1 : 0.0;
    }
    imu_msg.header.stamp = ros::Time::now();
    // Change the frame to "imu_link" for consistency with robot_localization.
    imu_msg.header.frame_id = "imu_link";
    return imu_msg;
}

// Convert an IMU message from NED to ENU frame.
// For vectors: [x, y, z] (NED) becomes [y, x, -z] (ENU). For orientation, we
// apply an equivalent rotation.
sensor_msgs::Imu convertNEDtoENU(const sensor_msgs::Imu& imu_ned) {
    sensor_msgs::Imu imu_enu = imu_ned;
    
    // Convert orientation using a transformation matrix.
    tf::Quaternion q_ned(imu_ned.orientation.x, imu_ned.orientation.y,
                           imu_ned.orientation.z, imu_ned.orientation.w);
    tf::Matrix3x3 R_ned;
    R_ned.setRotation(q_ned);
    // The conversion matrix from NED to ENU is:
    // [ 0  1  0 ]
    // [ 1  0  0 ]
    // [ 0  0 -1 ]
    tf::Matrix3x3 R_transform;
    R_transform.setValue(0, 1, 0,
                         1, 0, 0,
                         0, 0, -1);
    tf::Matrix3x3 R_enu = R_transform * R_ned;
    tf::Quaternion q_enu;
    R_enu.getRotation(q_enu);
    imu_enu.orientation.x = q_enu.x();
    imu_enu.orientation.y = q_enu.y();
    imu_enu.orientation.z = q_enu.z();
    imu_enu.orientation.w = q_enu.w();
    
    // Convert linear acceleration vector:
    imu_enu.linear_acceleration.x = imu_ned.linear_acceleration.y;
    imu_enu.linear_acceleration.y = imu_ned.linear_acceleration.x;
    imu_enu.linear_acceleration.z = -imu_ned.linear_acceleration.z;
    
    // Convert angular velocity vector:
    imu_enu.angular_velocity.x = imu_ned.angular_velocity.y;
    imu_enu.angular_velocity.y = imu_ned.angular_velocity.x;
    imu_enu.angular_velocity.z = -imu_ned.angular_velocity.z;
    
    // Update frame id to reflect use with robot_localization.
    imu_enu.header.frame_id = "imu_link";
    return imu_enu;
}

//----------------------------------------------------------------------
// ROS Callback for /cmd_vel
//----------------------------------------------------------------------

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    g_lastCmdVel = *msg;
    double v = g_lastCmdVel.linear.x;   // m/s
    double w = g_lastCmdVel.angular.z;    // rad/s
    double axleLength = DEFAULT_AXLE_LENGTH;
    
    // Compute left/right wheel velocities (m/s)
    double v_left  = v - (w * (axleLength / 2.0));
    double v_right = v + (w * (axleLength / 2.0));
    // Convert to angular velocities (rad/s)
    double omega_left  = v_left  / WHEEL_RADIUS;
    double omega_right = v_right / WHEEL_RADIUS;
    
    if (!g_robot->setVelocity_meterspersec(g_hSerial, v_left, v_right))
         ROS_ERROR("Failed to set linear velocities from /cmd_vel!");
    if (!g_robot->setVelocity_radianspersec(g_hSerial, omega_left, omega_right))
         ROS_ERROR("Failed to set angular velocities from /cmd_vel!");
    g_robot->forward(g_hSerial);
}

//----------------------------------------------------------------------
// Main Function
//----------------------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "combined_robot_node");
    ros::NodeHandle nh;
    
    // Publishers: odometry and IMU data.
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    ros::Publisher imu_pub  = nh.advertise<sensor_msgs::Imu>("/imu", 50);
    
    // Subscriber: velocity commands.
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
    
    // Initialize robot connection.
    g_robot = new lib0xRobotCpp();
    g_hSerial = g_robot->connect_comm(DEVICE_PORT);
    if (g_hSerial == nullptr) {
        ROS_ERROR("Failed to connect to the robot on %s!", DEVICE_PORT);
        return -1;
    }
    ROS_INFO("Connected to the robot on %s.", DEVICE_PORT);
    
    if (!g_robot->setMode(g_hSerial, 1)) {
        ROS_ERROR("Failed to set closed-loop velocity control mode!");
        return -1;
    }
    ROS_INFO("Motion control mode set to closed-loop velocity control (Mode 1).");
    
    if (!g_robot->setWheelDiameter_mm(g_hSerial, DEFAULT_WHEEL_DIAMETER * 1000.0))
        ROS_ERROR("Failed to set wheel diameter!");
    else
        ROS_INFO("Wheel diameter set to %.2f m", DEFAULT_WHEEL_DIAMETER);
    
    if (!g_robot->setRobotAxlelength_mm(g_hSerial, DEFAULT_AXLE_LENGTH * 1000.0))
        ROS_ERROR("Failed to set axle length!");
    else
        ROS_INFO("Axle length set to %.2f m", DEFAULT_AXLE_LENGTH);
    
    double distancePerTick = DISTANCE_PER_TICK;
    ROS_INFO("Distance per encoder tick: %.6f m", distancePerTick);
    
    // Stop the robot and reset encoder counts.
    g_robot->stop(g_hSerial);
    g_robot->resetMotorEncoderCount(g_hSerial);
    g_robot->setSafetyTimeout(g_hSerial, 0);
    g_robot->setSafety(g_hSerial, 0);
    
    // Get initial encoder values.
    g_robot->getLeftMotorCount(g_hSerial, &prevLeftEncoder);
    g_robot->getRightMotorCount(g_hSerial, &prevRightEncoder);
    
    // Setup IMU serial connection.
    serial::Serial imu_serial;
    std::string imu_port;
    nh.param<std::string>("imu_port", imu_port, "/dev/ttyIMU");
    try {
        imu_serial.setPort(imu_port);
        imu_serial.setBaudrate(57600);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        imu_serial.setTimeout(timeout);
        imu_serial.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open IMU serial port: %s", imu_port.c_str());
        ros::shutdown();
    }
    if (imu_serial.isOpen()) {
        ROS_INFO("Connected to IMU on port %s", imu_port.c_str());
    }
    // Allow time for IMU initialization.
    ros::Duration(5.0).sleep();
    imu_serial.write("#o0\n");  // Stop any ongoing stream
    ros::Duration(0.1).sleep();
    imu_serial.write("#ox\n");  // Set output mode to angles, acceleration, gyro
    ros::Duration(0.1).sleep();
    imu_serial.write("#f\n");   // Request data string
    ros::Duration(0.1).sleep();

    // Set initial odometry pose (input in meters and radians).
    std::cout << "Enter starting global X position (m): ";
    std::cin >> globalX;
    std::cout << "Enter starting global Y position (m): ";
    std::cin >> globalY;
    std::cout << "Enter starting heading (radians): ";
    std::cin >> globalTheta;
    
    tf::TransformBroadcaster odom_broadcaster;
    // A separate broadcaster for the IMU frame transform (base_link -> imu_link)
    tf::TransformBroadcaster imu_broadcaster;
    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
        // --------- IMU Data Processing -----------        
        if (imu_serial.available()) {
            std::string line = imu_serial.readline(256, "\n");
            if (line.find("#YPRAG=") != std::string::npos) {
                sensor_msgs::Imu imu_msg_ned = parseIMUData(line);
                sensor_msgs::Imu imu_msg_enu = convertNEDtoENU(imu_msg_ned);
                imu_pub.publish(imu_msg_enu);
            } else {
                ROS_WARN("Bad IMU data or sync issue");
            }
        }
        
        // --------- Odometry Update & Publishing -----------        
        updateRobotPosition(*g_robot, g_hSerial, globalX, globalY, globalTheta,
                            distancePerTick, DEFAULT_AXLE_LENGTH);
        
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = globalX;  // Already in meters
        odom.pose.pose.position.y = globalY;
        odom.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(globalTheta);
        odom.pose.pose.orientation = odom_quat;
        // Populate covariance matrices for pose and twist so that robot_localization can fuse them.
        // (Here only the x, y, and yaw components are non-zero; adjust values as needed.)
        for (int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        odom.pose.covariance[0] = 0.05;   // variance on x
        odom.pose.covariance[7] = 0.05;   // variance on y
        odom.pose.covariance[35] = 0.1;   // variance on yaw

        odom.twist.covariance[0] = 0.05;
        odom.twist.covariance[7] = 0.05;
        odom.twist.covariance[35] = 0.1;
        
        // Optionally, publish the last commanded velocities.
        odom.twist.twist = g_lastCmdVel;
        odom_pub.publish(odom);
        
        // Broadcast the transform over TF (odom -> base_link).
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = globalX;
        odom_trans.transform.translation.y = globalY;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // Publish a static transform between base_link and imu_link.
        geometry_msgs::TransformStamped imu_tf;
        imu_tf.header.stamp = ros::Time::now();
        imu_tf.header.frame_id = "base_link";
        imu_tf.child_frame_id = "imu_link";
        imu_tf.transform.translation.x = 0.0;
        imu_tf.transform.translation.y = 0.0;
        imu_tf.transform.translation.z = 0.0;
        imu_tf.transform.rotation = tf::createQuaternionMsgFromYaw(0);
        imu_broadcaster.sendTransform(imu_tf);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // Shutdown procedures.
    g_robot->stop(g_hSerial);
    if (!g_robot->disconnect_comm(g_hSerial))
        ROS_ERROR("Failed to disconnect from the robot!");
    else
        ROS_INFO("Disconnected from the robot.");
    
    delete g_robot;
    return 0;
}
