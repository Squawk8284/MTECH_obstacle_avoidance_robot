#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <cmath>
#include "0xRobotcpplib.h"  // Ensure this header is in your include path

using namespace std;

//---------------------------------------------------------
// Macros and Default Constants (for reference)
//---------------------------------------------------------
#define DEVICE_PORT "/dev/ttyRobot"  // Serial port used for robot control

// (The following values will be set on the robot via the provided functions.)
#define DEFAULT_WHEEL_DIAMETER_MM   (260.0)  // mm
#define DEFAULT_AXLE_LENGTH_MM      (590.0)  // mm
#define DEFAULT_ENCODER_TICKS       (2000)   // ticks per revolution

//---------------------------------------------------------
// Data Structures and Global Variables
//---------------------------------------------------------
// Simple 2D point (in mm)
struct Point {
    double x; // mm
    double y; // mm
};

// Bézier curve control points (in mm)
vector<Point> controlPoints = {
    {550, 550},          // Starting point (mm)
    {986.16, 1587.57},    // Control point 1 (mm)
    {1551.71, 2933.11},   // Control point 2 (mm)
    {2000, 4000}         // End point (mm)
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
    
    // Compute differences from previous counts.
    int32_t dLeft = currentLeftEncoder - prevLeftEncoder;
    int32_t dRight = currentRightEncoder - prevRightEncoder;
    
    // Update previous counts for next cycle.
    prevLeftEncoder = currentLeftEncoder;
    prevRightEncoder = currentRightEncoder;
    
    // Convert encoder ticks to distance (mm)
    double dLeft_mm = dLeft * distancePerTick;
    double dRight_mm = dRight * distancePerTick;
    
    // Compute the average forward displacement and change in orientation.
    double dCenter = (dLeft_mm + dRight_mm) / 2.0;
    double dTheta = (dRight_mm - dLeft_mm) / axleLength_mm;
    
    // Update the robot’s pose.
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
    
    // Debug message on one line.
    ROS_INFO("Error: dist=%.1f mm, thetaError=%.2f rad", distanceError, thetaError);
    
    // Controller gains (tunable)
    const double Kp_v = 0.5;  // m/s per m error
    const double Kp_w = 2.0;  // rad/s per rad error
    
    // Compute linear velocity v (convert mm error to m error)
    v = Kp_v * (distanceError / 1000.0);
    const double max_v = 0.3;  // m/s maximum
    if (v > max_v) v = max_v;
    
    // Compute angular velocity w
    w = Kp_w * thetaError;
    const double max_w = 1.0;  // rad/s maximum
    if (w > max_w)  w = max_w;
    if (w < -max_w) w = -max_w;
    
    ROS_INFO("Computed: v=%.3f m/s, w=%.3f rad/s", v, w);
}

//---------------------------------------------------------
// Main Control Loop: Follow Bézier Curve Using Velocity Commands
//---------------------------------------------------------
void followBezierCurve(lib0xRobotCpp &robot, void* hSerial, double distancePerTick)
{
    const double axleLength_mm = AXLE_LENGTH_MM;
    const double axleLength_m = AXLE_LENGTH_MM / 1000.0;
    
    // Get starting pose from the user.
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
    const double t_increment = 0.01;  // Increment when near target
    
    while (t <= 1.0 && ros::ok())
    {
        // Compute target point on the Bézier curve.
        Point target = computeBezierPoint(t, controlPoints);
        ROS_INFO("t=%.2f | Target=(%.1f, %.1f) mm", t, target.x, target.y);
        
        // Update current pose from encoders.
        updateRobotPosition(robot, hSerial, globalX, globalY, globalTheta, distancePerTick, axleLength_mm);
        
        // Compute control signals from current pose and target.
        double v, w;
        computeControlSignals(target.x, target.y, globalX, globalY, globalTheta, v, w);
        
        // Differential drive kinematics:
        //   v_left = v - (w * L/2)
        //   v_right = v + (w * L/2)
        double v_left = v - (w * (axleLength_m / 2.0));
        double v_right = v + (w * (axleLength_m / 2.0));
        
        // Also compute individual angular velocities (omega = v/r)
        double omega_left = v_left / (WHEEL_RADIUS_M);
        double omega_right = v_right / (WHEEL_RADIUS_M);
        
        ROS_INFO("Motor Commands: v_left=%.3f m/s, v_right=%.3f m/s, omega_left=%.3f rad/s, omega_right=%.3f rad/s",
                 v_left, v_right, omega_left, omega_right);
        
        // Command the robot using the provided functions.
        if (!robot.setVelocity_meterspersec(hSerial, v_left, v_right))
            ROS_ERROR("Failed to set linear velocities!");
        if (!robot.setVelocity_radianspersec(hSerial, omega_left, omega_right))
            ROS_ERROR("Failed to set angular velocities!");
        
        robot.forward(hSerial);
        ros::Duration(0.2).sleep();
        
        // Check distance to target.
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
    ROS_INFO("Starting robot movement node...");
    
    lib0xRobotCpp robot;
    
    // Connect to the robot.
    void* hSerial = robot.connect_comm(DEVICE_PORT);
    if (hSerial == nullptr) {
        ROS_ERROR("Failed to connect to the robot on %s!", DEVICE_PORT);
        return -1;
    }
    ROS_INFO("Connected to the robot on %s.", DEVICE_PORT);
    
    // Set wheel diameter and axle length using the provided functions.
    if (!robot.setWheelDiameter_mm(hSerial, DEFAULT_WHEEL_DIAMETER_MM))
        ROS_ERROR("Failed to set wheel diameter!");
    else
        ROS_INFO("Wheel diameter set to %.1f mm", DEFAULT_WHEEL_DIAMETER_MM);
    
    if (!robot.setRobotAxlelength_mm(hSerial, DEFAULT_AXLE_LENGTH_MM))
        ROS_ERROR("Failed to set robot axle length!");
    else
        ROS_INFO("Robot axle length set to %.1f mm", DEFAULT_AXLE_LENGTH_MM);
    
    // Retrieve encoder resolution (ticks per revolution).
    uint16_t ticks = 0;
    if (!robot.getEncoderTicksperRevolution(hSerial, &ticks))
        ROS_ERROR("Failed to get encoder ticks per revolution!");
    else {
        ROS_INFO("Encoder ticks per revolution: %d", ticks);
    }
    
    // Compute distance per encoder tick (in mm) using the set wheel diameter and retrieved ticks.
    double distancePerTick = (M_PI * DEFAULT_WHEEL_DIAMETER_MM) / ticks;
    ROS_INFO("Distance per encoder tick: %.4f mm", distancePerTick);
    
    // Initialize additional robot settings.
    robot.stop(hSerial);
    robot.resetMotorEncoderCount(hSerial);
    robot.setAcceleration(hSerial, 4);
    robot.setLinearVelocity_meterspersec(hSerial, 0.250);  // Nominal value; used internally.
    robot.setSafetyTimeout(hSerial, 0);
    robot.setSafety(hSerial, 0);
    
    // Follow the Bézier curve using continuous velocity commands.
    followBezierCurve(robot, hSerial, distancePerTick);
    
    if (!robot.disconnect_comm(hSerial))
        ROS_ERROR("Failed to disconnect from the robot!");
    else
        ROS_INFO("Disconnected from the robot.");
    
    return 0;
}
