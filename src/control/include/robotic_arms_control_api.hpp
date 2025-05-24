/**
 * @file robotic_arms_control_api.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief Robotic Arms Control for NEX Robot
 * @version 0.1
 * @date 2025-05-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __ROBOTICS_ARMS_CONTROL_APIS__H__
#define __ROBOTICS_ARMS_CONTROL_APIS__H__

#include <utils.hpp>

// ---------------------------------------------------------------------------
// Robot Arms control
// ---------------------------------------------------------------------------

/**
 * @brief Set the Robot Arm Position
 *
 * @param s Pointer to the serial object.
 * @param RoboticArmJointNumber Arm Joint Number
 * @param RobotArmAngle Angle by which RoboticArmJointNumber is to be rotated
 * @param RobotArmVelocity Velocity by which RoboticArmJointNumber is to be rotated
 * @return true if successful.
 * @return false if failed.
 */
bool setRobotArmPosition(serial::Serial *s, uint8_t RoboticArmJointNumber, int8_t RobotArmAngle, int8_t RobotArmVelocity)
{
    uint8_t buffer[3];
    buffer[0] = RoboticArmJointNumber;
    buffer[1] = static_cast<uint8_t>(RobotArmAngle);
    buffer[2] = static_cast<uint8_t>(RobotArmVelocity);

    if (!executeCommand(s, CMD(setRobotArmPosition, 0x26), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

#endif //__ROBOTICS_ARMS_CONTROL_APIS__H__