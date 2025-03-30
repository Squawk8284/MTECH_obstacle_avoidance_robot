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

// ---------------------------------------------------------------------------
// Function Defination
// ---------------------------------------------------------------------------

void init()
{
    setSafetyMode(robotPort,SAFETY_OFF);
    setRobotMode(robotPort,CLOSED_LOOP_CONTROL);
    setWheelDiameter_mm(robotPort, 260);
    setAxleLength_mm(robotPort, 590+80);
    setBuzzer(robotPort,BUZZER_OFF);
    
}
void CmdLinearVelocity_mps(float linearVelocity)
{
    setLeftMotorVelocity_mps(robotPort, linearVelocity);
    setRightMotorVelocity_mps(robotPort, linearVelocity);
    setRobotDirection(robotPort, FORWARD);
}

void CmdAngularVelocity_radps(float angularVelocity)
{
    setRobotAngularVelocityRadps(robotPort, angularVelocity);
    setRobotDirection(robotPort, FORWARD);
}

#endif //__USER_DEFINED__FUNCTIONS__