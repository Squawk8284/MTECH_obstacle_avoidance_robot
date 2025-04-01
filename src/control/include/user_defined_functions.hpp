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
#include <geometry_msgs/Twist.h>

// ---------------------------------------------------------------------------
// Extern Variables
// ---------------------------------------------------------------------------
extern serial::Serial *robotPort;
extern serial::Serial *imuPort;
extern float axel_length_m;
extern float wheel_dia_m;

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
}
void CmdLinearVelocity_mps(float linearVelocity, float angularVelocity)
{
    float leftVelocity = linearVelocity - ((axel_length_m * angularVelocity) / 2.0);
    float rightVelocity = linearVelocity + ((axel_length_m * angularVelocity) / 2.0);

    setLeftMotorVelocity_mps(robotPort, leftVelocity);
    setRightMotorVelocity_mps(robotPort, rightVelocity);
    setRobotDirection(robotPort, FORWARD);
}

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