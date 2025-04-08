/**
 * @file user_defined_functions.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief User Defined functions for relevant applications
 * @version 0.1
 * @date 2025-03-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __USER_DEFINED__FUNCTIONS__
#define __USER_DEFINED__FUNCTIONS__

#include <iostream>
#include <nex_robot.hpp>
#include <motor_control_api.hpp>
#include <power_management_api.hpp>
#include <inertial_control_api.hpp>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// ---------------------------------------------------------------------------
// Struct Variables
// ---------------------------------------------------------------------------
typedef struct
{
    double X;
    double Y;
    double Z;
    double Theta;
    float LinearVelocity;
    float AngularVelocity;
} Pose;

// ---------------------------------------------------------------------------
// Extern Variables
// ---------------------------------------------------------------------------
extern serial::Serial *robotPort;
extern serial::Serial *imuPort;
extern float axel_length_m;
extern float wheel_dia_m;
extern uint16_t CountsPerWheelRevolution;
extern double WheelRevPerCounts;
extern Pose robotPose;
constexpr double ALPHA = 0.98;

// ---------------------------------------------------------------------------
// Function Defination
// ---------------------------------------------------------------------------

void init()
{
    setSafetyMode(robotPort, SAFETY_OFF);
    setRobotMode(robotPort, CLOSED_LOOP_CONTROL);
    setWheelDiameter_mm(robotPort, WHEEL_DIA_IN_MM);
    setAxleLength_mm(robotPort, AXEL_LENGTH_IN_MM);
    setBuzzer(robotPort, BUZZER_OFF);
    getWheelDiameter_m(robotPort, &wheel_dia_m);
    getAxleLength_m(robotPort, &axel_length_m);
    clearEncoderCounts(robotPort);
    getEncoderResolutionCountPerWheelRevolution(robotPort, &CountsPerWheelRevolution);
    WheelRevPerCounts = static_cast<double>(1.0 / CountsPerWheelRevolution);
    PrintLn("Enter Starting X Pos:");
    std::cin >> robotPose.X;
    PrintLn("Enter Starting Y Pos:");
    std::cin >> robotPose.Y;
    PrintLn("Enter Starting 0 Pos:");
    std::cin >> robotPose.Theta;
}
void CmdLinearVelocity_mps(float linearVelocity, float angularVelocity)
{
    float leftVelocity = linearVelocity - ((axel_length_m * angularVelocity) / 2.0);
    float rightVelocity = linearVelocity + ((axel_length_m * angularVelocity) / 2.0);

    setLeftMotorVelocity_mps(robotPort, leftVelocity);
    setRightMotorVelocity_mps(robotPort, rightVelocity);
    setRobotDirection(robotPort, FORWARD);
}

void UpdateOdometry(geometry_msgs::TransformStamped &odom_trans, nav_msgs::Odometry &odom_msg)
{
    static int32_t prevLeftEncoderCounts = 0;
    static int32_t prevRightEncoderCounts = 0;
    ros::Time current_time = ros::Time::now();
    float LeftVelocity, RightVelocity;

    uint32_t currentLeftEncoderCounts, currentRightEncoderCounts;
    getLeftMotorEncoderCounts(robotPort, &currentLeftEncoderCounts);
    getRightMotorEncoderCounts(robotPort, &currentRightEncoderCounts);

    // Calculate delta (change in encoder counts)
    int32_t deltaLeft = currentLeftEncoderCounts - prevLeftEncoderCounts;
    int32_t deltaRight = currentRightEncoderCounts - prevRightEncoderCounts;

    // Update previous encoder counts for the next iteration
    prevLeftEncoderCounts = currentLeftEncoderCounts;
    prevRightEncoderCounts = currentRightEncoderCounts;

    double distLeft = (WheelRevPerCounts) * (deltaLeft) * (wheel_dia_m * M_PI);
    double distRight = (WheelRevPerCounts) * (deltaRight) * (wheel_dia_m * M_PI);

    float delta_centre = (distLeft + distRight) / 2.0;
    float delta_theta = (distRight - distLeft) / axel_length_m;

    robotPose.X += delta_centre * cos(robotPose.Theta + (delta_theta / 2.0));
    robotPose.Y += delta_centre * sin(robotPose.Theta + (delta_theta / 2.0));
    robotPose.Z = 0;
    robotPose.Theta += delta_theta;

    getLeftMotorVelocity_mps(robotPort, &LeftVelocity);
    getRightMotorVelocity_mps(robotPort, &RightVelocity);

    robotPose.LinearVelocity = (RightVelocity + LeftVelocity) / 2.0;
    robotPose.AngularVelocity = (LeftVelocity - RightVelocity) / axel_length_m;

    tf2::Quaternion q;
    q.setRPY(0, 0, robotPose.Theta);

    // Publish the TF transform from "odom" to "base_link"
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = robotPose.X;
    odom_trans.transform.translation.y = robotPose.Y;
    odom_trans.transform.translation.z = robotPose.Z;
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    // Publish the Odometry message on the /odom topic
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = robotPose.X;
    odom_msg.pose.pose.position.y = robotPose.Y;
    odom_msg.pose.pose.position.z = robotPose.Z;
    odom_msg.pose.pose.orientation = odom_trans.transform.rotation;
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = robotPose.LinearVelocity;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = robotPose.AngularVelocity;
}

// --------------------- Functions for calculations of YAW from internal Sensor ---------------------
void computeGravityAndRemove(double ax, double ay, double az, double &g_x, double &g_y, double &g_z)
{
    // Compute gravity vector
    double g = sqrt(ax * ax + ay * ay + az * az);

    g_x = g * sin(atan2(-ax, sqrt(ay * ay + az * az)));                       // g_x
    g_y = -g * sin(atan2(ay, az)) * cos(atan2(-ax, sqrt(ay * ay + az * az))); // g_y
    g_z = g * cos(atan2(ay, az)) * cos(atan2(-ax, sqrt(ay * ay + az * az)));  // g_z
}

double computeYaw(double mx, double my, double mz, double roll, double pitch)
{
    double yaw = 0.0;

    if (mx != 0 || my != 0 || mz != 0)
    { // Ensure magnetometer data is valid
        // Correct magnetometer readings for roll and pitch
        double m_x = mx * cos(pitch) + mz * sin(pitch);
        double m_y = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

        // Compute yaw
        yaw = atan2(-m_y, m_x);
    }
    return yaw;
}

void computeRollPitch(double ax, double ay, double az, double &roll, double &pitch)
{
    // Roll and Pitch using accelerometer
    roll = atan2(ay, az);
    pitch = atan2(-ax, sqrt(ay * ay + az * az));
}

bool getYPRG(double *yaw, double *pitch, double *roll, double gravity[3])
{
    float xAccel, yAccel, zAccel;
    float xGyro, yGyro, zGyro;
    float xMag, yMag, zMag;

    if (!get3AxisAccelorometer(robotPort, &xAccel, &yAccel, &zAccel))
        return FAILURE;
    if (!get3AxisGyroscope(robotPort, &xGyro, &yGyro, &zGyro))
        return FAILURE;
    if (!get3AxisMagnetometer(robotPort, &xMag, &yMag, &zMag))
        return FAILURE;

    static double prevRoll = 0.0, prevPitch = 0.0, prevYaw = 0.0;

    double tempRoll, tempPitch;
    computeRollPitch(xAccel, yAccel, zAccel, tempRoll, tempPitch);
    *roll = ALPHA * (prevRoll + xGyro) + (1 - ALPHA) * tempRoll;
    *pitch = ALPHA * (prevPitch + yGyro) + (1 - ALPHA) * tempPitch;

    prevRoll = *roll;
    prevPitch = *pitch;

    *yaw = computeYaw(xMag, yMag, zMag, *roll, *pitch);
    *yaw = ALPHA * (prevYaw + zGyro) + (1 - ALPHA) * (*yaw);
    prevYaw = *yaw;

    double g_x, g_y, g_z;
    computeGravityAndRemove(xAccel, yAccel, zAccel, g_x, g_y, g_z);
    gravity[0] = g_x;
    gravity[1] = g_y;
    gravity[2] = g_z;

    return SUCCESS;
}

#endif //__USER_DEFINED__FUNCTIONS__