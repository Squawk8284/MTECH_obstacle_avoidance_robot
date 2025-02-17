#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <cmath>
#include "std_msgs/Bool.h"          // For shutdown signal
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>        // For IMU messages
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "0xRobotcpplib.h"          // Ensure this header is in your include path

using namespace std;

//----------------------------------------------------------------------
// Global Variables for IMU Subscription (using callback)
//----------------------------------------------------------------------
sensor_msgs::Imu latest_imu;
bool imu_received = false;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    latest_imu = *msg;
    imu_received = true;
}

//----------------------------------------------------------------------
// Macros and Constants for Odometry & Path Following
//----------------------------------------------------------------------

#define DEVICE_PORT "/dev/ttyRobot"  // Serial port for robot control

// Default hardware values.
#define DEFAULT_WHEEL_DIAMETER_MM   (260.0)    // in mm
#define DEFAULT_AXLE_LENGTH_MM      (700.0)    // in mm
#define DEFAULT_ENCODER_TICKS       (3840)     // ticks per revolution

// Define wheel radius in meters.
#define WHEEL_RADIUS_M  (DEFAULT_WHEEL_DIAMETER_MM / 2.0 / 1000.0)

// Distance per encoder tick in mm.
#define DISTANCE_PER_TICK_IN_MM  ((M_PI * DEFAULT_WHEEL_DIAMETER_MM) / (DEFAULT_ENCODER_TICKS))

// Controller threshold: if the robot is within this distance of the target, advance t.
#define DIST_THRESHOLD   (500.0)  // mm

// t_increment for advancing the Bézier parameter t.
#define T_INC       (0.01)

//----------------------------------------------------------------------
// Macros for Control Gains (for easy tuning)
//----------------------------------------------------------------------
#define KP_V 0.5   // Linear gain (m/s per meter error)
#define KP_W 1.5   // Angular gain (rad/s per rad error)

//----------------------------------------------------------------------
// Data Structures for Path Following
//----------------------------------------------------------------------

struct Point {
    double x; // in mm
    double y; // in mm
};

// Updated Bézier curve control points (in mm)
std::vector<Point> controlPoints = {
    {600,600},             // Start
    {4937.43,619.92},       // Control point 1
    {4607.53,1783.65},      // Control point 2
    {6900,4200}            // End
};

//----------------------------------------------------------------------
// Bézier Curve Computation Functions
//----------------------------------------------------------------------

unsigned long factorial(unsigned int n) {
    unsigned long result = 1;
    for (unsigned int i = 2; i <= n; i++) {
        result *= i;
    }
    return result;
}

unsigned long binomialCoeff(unsigned int n, unsigned int i) {
    return factorial(n) / (factorial(i) * factorial(n - i));
}

Point computeBezierPoint(double t, const std::vector<Point>& ctrlPts) {
    int n = ctrlPts.size() - 1;
    Point result = {0.0, 0.0};
    for (int i = 0; i <= n; i++) {
        double bernstein = binomialCoeff(n, i) * pow(1 - t, n - i) * pow(t, i);
        result.x += bernstein * ctrlPts[i].x;
        result.y += bernstein * ctrlPts[i].y;
    }
    return result;
}

//----------------------------------------------------------------------
// Encoder-based Odometry Functions
//----------------------------------------------------------------------

// (Assuming that your hardware odometry is provided in the standard ROS frame: 
// x forward, y left, with heading measured counterclockwise.)
int32_t prevLeftEncoder = 0;
int32_t prevRightEncoder = 0;

void updateRobotPosition(lib0xRobotCpp &robot, void* hSerial,
                           double &x, double &y, double &theta,
                           double distancePerTick, double axleLength_mm) {
    int32_t currentLeftEncoder = 0;
    int32_t currentRightEncoder = 0;
    
    robot.getLeftMotorCount(hSerial, &currentLeftEncoder);
    robot.getRightMotorCount(hSerial, &currentRightEncoder);
    
    ROS_INFO("Encoders: Left=%d, Right=%d", currentLeftEncoder, currentRightEncoder);
    
    int32_t dLeft = currentLeftEncoder - prevLeftEncoder;
    int32_t dRight = currentRightEncoder - prevRightEncoder;
    
    prevLeftEncoder = currentLeftEncoder;
    prevRightEncoder = currentRightEncoder;
    
    double dLeft_mm = dLeft * DISTANCE_PER_TICK_IN_MM;
    double dRight_mm = dRight * DISTANCE_PER_TICK_IN_MM;
    
    double dCenter = (dLeft_mm + dRight_mm) / 2.0;
    double dTheta = (dRight_mm - dLeft_mm) / axleLength_mm;
    
    double thetaMid = theta + dTheta / 2.0;
    x += dCenter * cos(thetaMid);
    y += dCenter * sin(thetaMid);
    theta += dTheta;
    
    while (theta > M_PI)  theta -= 2 * M_PI;
    while (theta < -M_PI) theta += 2 * M_PI;
    
    ROS_INFO("Pose (odometry): x=%.1f mm, y=%.1f mm, theta=%.2f rad", x, y, theta);
}

//----------------------------------------------------------------------
// Control Signal Computation (Tuned Gains, max velocities 0.4)
//----------------------------------------------------------------------

void computeControlSignals(double targetX, double targetY,
                           double currentX, double currentY, double currentTheta,
                           double &v, double &w) {
    double errorX = targetX - currentX;
    double errorY = targetY - currentY;
    // Compute desired heading in standard ROS frame.
    double desiredTheta = atan2(errorY, errorX);
    double thetaError = desiredTheta - currentTheta;
    
    while (thetaError > M_PI)  thetaError -= 2 * M_PI;
    while (thetaError < -M_PI) thetaError += 2 * M_PI;
    
    double distanceError = sqrt(errorX * errorX + errorY * errorY);
    ROS_INFO("Control Error: dist=%.1f mm, thetaError=%.2f rad", distanceError, thetaError);
    
    double Kp_v = KP_V;
    double Kp_w = KP_W;
    
    v = Kp_v * (distanceError / 1000.0);
    const double max_v = 0.4;
    if (v > max_v) v = max_v;
    
    w = Kp_w * thetaError;
    const double max_w = 0.4;
    if (w > max_w)  w = max_w;
    if (w < -max_w) w = -max_w;
    
    ROS_INFO("Computed Control: v=%.3f m/s, w=%.3f rad/s", v, w);
}

//----------------------------------------------------------------------
// Main Control Loop: Follow Bézier Curve with IMU Fusion (Using ROS Subscription)
//----------------------------------------------------------------------

void followBezierCurve(lib0xRobotCpp &robot, void* hSerial, double distancePerTick) {
    const double axleLength_mm = DEFAULT_AXLE_LENGTH_MM;
    const double axleLength_m = axleLength_mm / 1000.0;
    
    // Ask for starting pose (in the standard ROS frame).
    double globalX, globalY, globalTheta;
    std::cout << "Enter starting global X position (mm): ";
    std::cin >> globalX;
    std::cout << "Enter starting global Y position (mm): ";
    std::cin >> globalY;
    std::cout << "Enter starting heading (radians): ";
    std::cin >> globalTheta;
    
    robot.resetMotorEncoderCount(hSerial);
    robot.getLeftMotorCount(hSerial, &prevLeftEncoder);
    robot.getRightMotorCount(hSerial, &prevRightEncoder);
    
    ros::Duration(1.0).sleep();
    
    double t = 0.0; // Bézier parameter
    const double t_increment = T_INC;
    
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    
    const double headingAlpha = 0.09; // Base complementary filter constant for IMU fusion.
    
    while (t <= (1 + (5 * T_INC)) && ros::ok()) {
        Point P1 = computeBezierPoint(t, controlPoints);
        double t_next = (t + t_increment > 1.0) ? 1.0 : t + t_increment;
        Point P2 = computeBezierPoint(t_next, controlPoints);
        
        double dx = P1.x - globalX;
        double dy = P1.y - globalY;
        double distError = sqrt(dx * dx + dy * dy);
        
        double alpha = 1.0 - std::fmin(distError / DIST_THRESHOLD, 1.0);
        Point smoothTarget;
        smoothTarget.x = (1 - alpha) * P1.x + alpha * P2.x;
        smoothTarget.y = (1 - alpha) * P1.y + alpha * P2.y;
        
        ROS_INFO("t=%.2f | P1=(%.1f, %.1f), P2=(%.1f, %.1f)", t, P1.x, P1.y, P2.x, P2.y);
        ROS_INFO("Alpha=%.2f | Smoothed Target=(%.1f, %.1f) mm", alpha, smoothTarget.x, smoothTarget.y);
        
        updateRobotPosition(robot, hSerial, globalX, globalY, globalTheta, distancePerTick, axleLength_mm);
        
        // Fuse IMU heading using the latest received message.
        if (imu_received) {
            // Do NOT negate the IMU reading.
            double imu_yaw = tf::getYaw(latest_imu.orientation);
            double yawCovariance = latest_imu.orientation_covariance[8];
            double effectiveHeadingAlpha = headingAlpha;
            if (yawCovariance > 0.0) {
                effectiveHeadingAlpha = headingAlpha / (1.0 + yawCovariance);
                if (effectiveHeadingAlpha < 0.05)
                    effectiveHeadingAlpha = 0.05;
            }
            globalTheta = effectiveHeadingAlpha * imu_yaw + (1 - effectiveHeadingAlpha) * globalTheta;
            ROS_INFO("IMU yaw: %.2f, Covariance: %.3f, Effective Alpha: %.3f, Fused Theta: %.2f",
                     imu_yaw, yawCovariance, effectiveHeadingAlpha, globalTheta);
        } else {
            ROS_WARN("No IMU data received yet.");
        }
        
        // Publish odometry in the ROS standard frame.
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = globalX / 1000.0;
        odom.pose.pose.position.y = globalY / 1000.0;
        odom.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(globalTheta);
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;
        odom_pub.publish(odom);
        
        double v, w;
        computeControlSignals(smoothTarget.x, smoothTarget.y, globalX, globalY, globalTheta, v, w);
        ROS_INFO("Control: v=%.3f m/s, w=%.3f rad/s", v, w);
        
        double v_left = v - (w * (axleLength_m / 2.0));
        double v_right = v + (w * (axleLength_m / 2.0));
        double omega_left = v_left / WHEEL_RADIUS_M;
        double omega_right = v_right / WHEEL_RADIUS_M;
        
        ROS_INFO("Motor Commands: v_left=%.3f m/s, v_right=%.3f m/s, omega_left=%.3f rad/s, omega_right=%.3f rad/s",
                 v_left, v_right, omega_left, omega_right);
        
        if (!robot.setVelocity_meterspersec(hSerial, v_left, v_right))
            ROS_ERROR("Failed to set linear velocities!");
        if (!robot.setVelocity_radianspersec(hSerial, omega_left, omega_right))
            ROS_ERROR("Failed to set angular velocities!");
        
        robot.forward(hSerial);
        
        ros::Duration(0.2).sleep();
        
        if (distError < DIST_THRESHOLD) {
            t += t_increment;
            ROS_INFO("Advancing to next target: t=%.2f", t);
        }
        
        ros::spinOnce();
    }
    
    robot.stop(hSerial);
    ROS_INFO("Robot stopped after following the curve.");
}

//----------------------------------------------------------------------
// Main Function
//----------------------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    
    // Subscribe to the "imu" topic.
    ros::Subscriber imu_sub = nh.subscribe("imu", 10, imuCallback);
    
    ros::Publisher shutdown_pub = nh.advertise<std_msgs::Bool>("/shutdown_signal", 1);
    ROS_INFO("Starting robot movement node...");
    
    lib0xRobotCpp robot;
    
    // Connect to the robot.
    void* hSerial = robot.connect_comm(DEVICE_PORT);
    if (hSerial == nullptr) {
        ROS_ERROR("Failed to connect to the robot on %s!", DEVICE_PORT);
        return -1;
    }
    ROS_INFO("Connected to the robot on %s.", DEVICE_PORT);
    
    if (!robot.setMode(hSerial, 1)) {
        ROS_ERROR("Failed to set motion control mode to closed-loop velocity control!");
        return -1;
    } else {
        ROS_INFO("Motion control mode set to closed-loop velocity control (Mode 1).");
    }
    
    if (!robot.setWheelDiameter_mm(hSerial, DEFAULT_WHEEL_DIAMETER_MM))
        ROS_ERROR("Failed to set wheel diameter!");
    else
        ROS_INFO("Wheel diameter set to %.1f mm", DEFAULT_WHEEL_DIAMETER_MM);
    
    if (!robot.setRobotAxlelength_mm(hSerial, DEFAULT_AXLE_LENGTH_MM))
        ROS_ERROR("Failed to set axle length!");
    else
        ROS_INFO("Axle length set to %.1f mm", DEFAULT_AXLE_LENGTH_MM);
    
    double distancePerTick = (M_PI * DEFAULT_WHEEL_DIAMETER_MM) / DEFAULT_ENCODER_TICKS;
    ROS_INFO("Distance per encoder tick: %.4f mm", distancePerTick);
    
    robot.stop(hSerial);
    robot.resetMotorEncoderCount(hSerial);
    robot.setSafetyTimeout(hSerial, 0);
    robot.setSafety(hSerial, 0);
    
    followBezierCurve(robot, hSerial, distancePerTick);
    
    if (!robot.disconnect_comm(hSerial))
        ROS_ERROR("Failed to disconnect from the robot!");
    else
        ROS_INFO("Disconnected from the robot.");
    
    std_msgs::Bool shutdown_msg;
    shutdown_msg.data = true;
    shutdown_pub.publish(shutdown_msg);
    ROS_INFO("Shutdown signal sent. Terminating control node.");
    
    return 0;
}
