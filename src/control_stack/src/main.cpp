#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <cmath>
#include "std_msgs/Bool.h"  // For shutdown signal
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "0xRobotcpplib.h"  // Ensure this header is in your include path

using namespace std;

//---------------------------------------------------------
// Macros and Constants
//---------------------------------------------------------
#define DEVICE_PORT "/dev/ttyRobot"  // Serial port for robot control

// Default hardware values.
#define DEFAULT_WHEEL_DIAMETER_MM   (260.0)    // in mm
#define DEFAULT_AXLE_LENGTH_MM      (590.0)    // in mm
#define DEFAULT_ENCODER_TICKS       (2000)     // ticks per revolution

// Define wheel radius in meters.
#define WHEEL_RADIUS_M  (DEFAULT_WHEEL_DIAMETER_MM / 2.0 / 1000.0)

// Distance per encoder tick in mm = (PI * wheel_diameter) / ticks
#define DISTANCE_PER_TICK_IN_MM  ((M_PI * DEFAULT_WHEEL_DIAMETER_MM) / (DEFAULT_ENCODER_TICKS))

// Controller threshold: if the robot is within this distance of the target, advance to the next target.
#define DIST_THRESHOLD   (500.0)  // mm

#define T_INC       (0.04)

//---------------------------------------------------------
// Data Structures
//---------------------------------------------------------
struct Point {
    double x; // in mm
    double y; // in mm
};

// Bézier curve control points (in mm)
vector<Point> controlPoints = {
    {600, 600},         // Start
    {2440.64, 845.26},    // Control point 1
    {3902.44, 42.4},   // Control point 2
    {6300, 3600}         // End
};

//---------------------------------------------------------
// Bézier Curve Computation Functions
//---------------------------------------------------------
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

Point computeBezierPoint(double t, const vector<Point>& ctrlPts) {
    int n = ctrlPts.size() - 1;
    Point result = {0.0, 0.0};
    for (int i = 0; i <= n; i++) {
        double bernstein = binomialCoeff(n, i) * pow(1 - t, n - i) * pow(t, i);
        result.x += bernstein * ctrlPts[i].x;
        result.y += bernstein * ctrlPts[i].y;
    }
    return result;
}

//---------------------------------------------------------
// Robot Localization (Dead-Reckoning Using Encoders)
//---------------------------------------------------------
int32_t prevLeftEncoder = 0;
int32_t prevRightEncoder = 0;

void updateRobotPosition(lib0xRobotCpp &robot, void* hSerial,
                           double &x, double &y, double &theta,
                           double distancePerTick, double axleLength_mm)
{
    int32_t currentLeftEncoder = 0;
    int32_t currentRightEncoder = 0;
    
    // Retrieve current encoder counts.
    robot.getLeftMotorCount(hSerial, &currentLeftEncoder);
    robot.getRightMotorCount(hSerial, &currentRightEncoder);
    
    // Print encoder counts on one line.
    ROS_INFO("Encoders: Left=%d, Right=%d", currentLeftEncoder, currentRightEncoder);
    
    // Compute difference in encoder counts.
    int32_t dLeft = currentLeftEncoder - prevLeftEncoder;
    int32_t dRight = currentRightEncoder - prevRightEncoder;
    
    // Update previous counts.
    prevLeftEncoder = currentLeftEncoder;
    prevRightEncoder = currentRightEncoder;
    
    // Convert encoder ticks to distance (mm)
    double dLeft_mm = dLeft * distancePerTick;
    double dRight_mm = dRight * distancePerTick;
    
    // Compute average displacement and orientation change.
    double dCenter = (dLeft_mm + dRight_mm) / 2.0;
    double dTheta = (dRight_mm - dLeft_mm) / axleLength_mm;
    
    // Update global pose using average heading.
    double thetaMid = theta + dTheta / 2.0;
    x += dCenter * cos(thetaMid);
    y += dCenter * sin(thetaMid);
    theta += dTheta;
    
    // Normalize theta to (-pi, pi)
    while (theta > M_PI)  theta -= 2 * M_PI;
    while (theta < -M_PI) theta += 2 * M_PI;
    
    ROS_INFO("Pose: x=%.1f mm, y=%.1f mm, theta=%.2f rad", x, y, theta);
}

//---------------------------------------------------------
// Control Law: Compute v and w from Pose Error
//---------------------------------------------------------
void computeControlSignals(double targetX, double targetY,
                           double currentX, double currentY, double currentTheta,
                           double &v, double &w)
{
    // Compute error vector (in mm)
    double errorX = targetX - currentX;
    double errorY = targetY - currentY;
    double distanceError = sqrt(errorX * errorX + errorY * errorY);
    double desiredTheta = atan2(errorY, errorX);
    double thetaError = desiredTheta - currentTheta;
    while (thetaError > M_PI)  thetaError -= 2 * M_PI;
    while (thetaError < -M_PI) thetaError += 2 * M_PI;
    
    // Print error in one line.
    ROS_INFO("Error: dist=%.1f mm, thetaError=%.2f rad", distanceError, thetaError);
    
    // Proportional controller gains (tunable)
    const double Kp_v = 1.5;  // m/s per m error
    const double Kp_w = 2.0;  // rad/s per rad error
    
    // Compute linear velocity v (convert mm error to m error)
    v = Kp_v * (distanceError / 1000.0);
    const double max_v = 10.0;  // m/s maximum
    if (v > max_v) v = max_v;
    
    // Compute angular velocity w.
    w = Kp_w * thetaError;
    const double max_w = 10.0;  // rad/s maximum
    if (w > max_w)  w = max_w;
    if (w < -max_w) w = -max_w;
    
    ROS_INFO("Computed: v=%.3f m/s, w=%.3f rad/s", v, w);
}

//---------------------------------------------------------
// Main Control Loop: Follow Bézier Curve Using Velocity Commands
//---------------------------------------------------------
void followBezierCurve(lib0xRobotCpp &robot, void* hSerial, double distancePerTick)
{
    const double axleLength_mm = DEFAULT_AXLE_LENGTH_MM;
    const double axleLength_m = axleLength_mm / 1000.0;
    
    // Get starting pose from user.
    double globalX, globalY, globalTheta;
    cout << "Enter starting global X position (mm): ";
    cin >> globalX;
    cout << "Enter starting global Y position (mm): ";
    cin >> globalY;
    cout << "Enter starting heading (radians): ";
    cin >> globalTheta;
    
    // Reset encoder counts.
    robot.resetMotorEncoderCount(hSerial);
    robot.getLeftMotorCount(hSerial, &prevLeftEncoder);
    robot.getRightMotorCount(hSerial, &prevRightEncoder);
    
    ros::Duration(1.0).sleep();
    
    double t = 0.0;
    const double t_increment = T_INC;  // Increment t when near target

    // Create a ROS publisher for odometry.
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    
    while (t <= 1.0 && ros::ok())
    {
        // Compute the target point on the Bézier curve.
        Point target = computeBezierPoint(t, controlPoints);
        ROS_INFO("t=%.2f | Target=(%.1f, %.1f) mm", t, target.x, target.y);
        
        // Update current pose from encoders.
        updateRobotPosition(robot, hSerial, globalX, globalY, globalTheta, distancePerTick, axleLength_mm);

        // Publish odometry.
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        // Convert mm to m for publishing.
        odom.pose.pose.position.x = globalX / 1000.0;
        odom.pose.pose.position.y = globalY / 1000.0;
        odom.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(globalTheta);
        odom.pose.pose.orientation = odom_quat;
        // Twist is left zero.
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;
        odom_pub.publish(odom);
        
        // Compute control signals.
        double v, w;
        computeControlSignals(target.x, target.y, globalX, globalY, globalTheta, v, w);
        
        ROS_INFO("Control: v=%.3f m/s, w=%.3f rad/s", v, w);
        
        // Convert v and w into individual motor commands using differential drive:
        //   v_left = v - (w * (L/2))
        //   v_right = v + (w * (L/2))
        double v_left = v - (w * (axleLength_m / 2.0));
        double v_right = v + (w * (axleLength_m / 2.0));
        
        // Compute corresponding angular velocities (omega = v / r).
        double omega_left = v_left / WHEEL_RADIUS_M;
        double omega_right = v_right / WHEEL_RADIUS_M;
        
        ROS_INFO("Motor Commands: v_left=%.3f m/s, v_right=%.3f m/s, omega_left=%.3f rad/s, omega_right=%.3f rad/s",
                 v_left, v_right, omega_left, omega_right);
        
        // Set the motor velocities using the provided functions.
        if (!robot.setVelocity_meterspersec(hSerial, v_left, v_right))
            ROS_ERROR("Failed to set linear velocities!");
        if (!robot.setVelocity_radianspersec(hSerial, omega_left, omega_right))
            ROS_ERROR("Failed to set angular velocities!");
        
        // IMPORTANT: Issue a forward command after setting velocities.
        robot.forward(hSerial);
        
        ros::Duration(0.2).sleep();
        
        // Check if the robot is near the target.
        double dx = target.x - globalX;
        double dy = target.y - globalY;
        double distError = sqrt(dx * dx + dy * dy);
        ROS_INFO("Distance to target: %.1f mm", distError);
        if (distError < DIST_THRESHOLD) {
            t += t_increment;
            ROS_INFO("Advancing to next target: t=%.2f", t);
        }
        
        ros::spinOnce();
    }
    
    robot.stop(hSerial);
    ROS_INFO("Robot stopped after following the curve.");
}

//---------------------------------------------------------
// Main Function
//---------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
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
    
    // Set the motion control mode to closed-loop velocity control (Mode 1).
    if (!robot.setMode(hSerial, 1)) {
        ROS_ERROR("Failed to set motion control mode to closed-loop velocity control!");
        return -1;
    }
    else {
        ROS_INFO("Motion control mode set to closed-loop velocity control (Mode 1).");
    }
    
    // Configure hardware parameters.
    if (!robot.setWheelDiameter_mm(hSerial, DEFAULT_WHEEL_DIAMETER_MM))
        ROS_ERROR("Failed to set wheel diameter!");
    else
        ROS_INFO("Wheel diameter set to %.1f mm", DEFAULT_WHEEL_DIAMETER_MM);
    
    if (!robot.setRobotAxlelength_mm(hSerial, DEFAULT_AXLE_LENGTH_MM))
        ROS_ERROR("Failed to set axle length!");
    else
        ROS_INFO("Axle length set to %.1f mm", DEFAULT_AXLE_LENGTH_MM);
    
    // Compute distance per encoder tick (mm) using the default encoder ticks.
    double distancePerTick = (M_PI * DEFAULT_WHEEL_DIAMETER_MM) / DEFAULT_ENCODER_TICKS;
    ROS_INFO("Distance per encoder tick: %.4f mm", distancePerTick);
    
    // Additional robot initialization.
    robot.stop(hSerial);
    robot.resetMotorEncoderCount(hSerial);
    // robot.setAcceleration(hSerial, 4);
    // robot.setLinearVelocity_meterspersec(hSerial, 0.250);  // Nominal value; used internally.
    robot.setSafetyTimeout(hSerial, 0);
    robot.setSafety(hSerial, 0);
    
    followBezierCurve(robot, hSerial, distancePerTick);
    
    if (!robot.disconnect_comm(hSerial))
        ROS_ERROR("Failed to disconnect from the robot!");
    else
        ROS_INFO("Disconnected from the robot.");
    
    // Send shutdown signal to Python node
    std_msgs::Bool shutdown_msg;
    shutdown_msg.data = true;
    shutdown_pub.publish(shutdown_msg);
    ROS_INFO("Shutdown signal sent. Terminating control node.");

    return 0;
}
