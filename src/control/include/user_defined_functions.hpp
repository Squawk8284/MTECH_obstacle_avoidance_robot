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

#endif //__USER_DEFINED__FUNCTIONS__