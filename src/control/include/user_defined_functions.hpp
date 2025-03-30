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

#include <nex_robot.hpp>
#include <iostream>
#include <nex_robot.hpp>
#include <motor_control_api.hpp>
#include <power_management_api.hpp>
#include <inertial_control_api.hpp>

void code();

void init(serial::Serial *RobotSerial, serial::Serial *IMUSerial = nullptr)
{
    
}

void setLinearVelocityMPS(serial::Serial *RobotSerial, float linearVelocity)
{
    setLeftMotorVelocity_mps(RobotSerial, linearVelocity);
    setRightMotorVelocity_mps(RobotSerial, linearVelocity);
    setRobotDirection(RobotSerial,FORWARD);
}

void setAngularVelocityRadps(serial::Serial *RobotSerial, float angularVelocity)
{
    setAngularVelocityRadps(RobotSerial,angularVelocity);
    setRobotDirection(RobotSerial,FORWARD);
}



#endif //__USER_DEFINED__FUNCTIONS__