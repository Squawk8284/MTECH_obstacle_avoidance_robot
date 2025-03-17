/**
 * @file motor_control_api.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief Motor control api for nex robot
 * @include nex_robot.hpp file for various functionality
 * @version 0.1
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __MOTOR_CONTORL_APIS__
#define __MOTOR_CONTORL_APIS__

#include <nex_robot.hpp>

#define SAFETY_ON (1)
#define SAFETY_OFF (0)

#define OPEN_LOOP_CONTROL (0)
#define CLOSED_LOOP_CONTROL (1)
#define POSITION_CONTROL (2)
// ------------------------------------
// Motor Control Commands
// ------------------------------------

/**
 * @brief Set safety timeout (in seconds); expected response length: 3 bytes.
 *
 * @param s
 * @param timeout
 * @return true
 * @return false
 */
bool setSafetyTimeout(serial::Serial *s, uint8_t timeout)
{
    return executeCommand(s, 0x7A, {0x01, timeout}, 3);
}

/**
 * @brief Get safety timeout; expected response length: 4 bytes. Gives back hex value
 *
 * @param s
 * @param outData
 * @return true
 * @return false
 */
bool getSafetyTimeout(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x7A, {0x02}, 4, outData);
}

/**
 * @brief Set safety ON/OFF (1 for ON, 0 for OFF); expected response length: 3 bytes.
 *
 * Behavior during robot safety ON feature1.
 *
 * Max velocity of robot limited to a safe value. This safe max velocity for 0X Delta robot is 0.4 m/Sec.
 *
 * Behavior during robot safety OFF feature.
 *
 * Max velocity of robot can be altered up to robot's maximum reachable velocity
 *
 * @param s
 * @param mode
 * @return true
 * @return false
 */
bool setSafetyMode(serial::Serial *s, uint8_t mode)
{
    return executeCommand(s, 0x89, {mode}, 3);
}

/**
 * @brief Set mode (0: open-loop, 1: closed-loop, 2: position); expected response length: 3 bytes.
 *
 * In position control mode, every new position command will reset the encoder count
 * registers of both the motors and the target encoder count will be achieved by starting from
 * zero.
 * @param s
 * @param mode
 * @return true
 * @return false
 */
bool setRobotMode(serial::Serial *s, uint8_t mode)
{
    return executeCommand(s, 0x90, {mode}, 3);
}

/**
 * @brief Get mode; expected response length: 4 bytes.
 *
 * (*outData) = 0 - Open loop control
 *
 * (*outData) = 1 - Closed loop Control
 *
 * (*outData) = 2 - Position control
 * @param s
 * @param outData
 * @return true
 * @return false
 */
bool getRobotMode(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x91, {0x00}, 4, outData);
}

/**
 * @brief Set acceleration (1-16); expected response length: 3 bytes.
 *
 * Note: Acceleration count (n) must be in the range of 1-16.
 * Acceleration = ( 168.72 * n) mm/Sec squared
 * @param s
 * @param acc
 * @return true
 * @return false
 */
bool setAcceleration(serial::Serial *s, uint8_t acc)
{
    return executeCommand(s, 0x98, {acc}, 3);
}

/**
 * @brief Get acceleration; expected response length: 4 bytes.
 *
 * Gives the result in mm/sec^2
 * @param s
 * @param outData
 * @return true
 * @return false
 */
bool getAcceleration(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x99, {0x00}, 4, outData);
}

/**
 * @brief Sets the velocity in range of 512 to -512 depending on range of allowed velocity for safety on/off
 *
 * Set left motor velocity (raw, two bytes); expected response length: 3 bytes.
 *
 * Note: Velocity = 512-Full forward,
 * = 0-Stop,
 * = -512-Full reverse
 *
 * Robot direction command will act exactly opposite if motor velocity is set to -ve value.
 * e.g if velocity set for motor is -ve, then sending forward direction command will move the
 * motor in reverse direction.
 *
 * Any change in velocity of robot will reflect only and only after resending robot
 * direction command.
 *
 * @param s
 * @param velocity
 * @return true
 * @return false
 */
bool setLeftMotorVelocity(serial::Serial *s, int16_t *velocity)
{
    uint8_t msb = static_cast<uint8_t>(*velocity >> 8);
    uint8_t lsb = static_cast<uint8_t>(*velocity & 0xFF);
    return executeCommand(s, 0x95, {msb, lsb}, 3);
}

/**
 * @brief Get left motor velocity (raw); expected response length: 5 bytes.
 *
 * Returns the velocity in range of 512 to -512 depending on range of allowed velocity for safety on/off
 *
 * @param s
 * @param velocity
 * @return true
 * @return false
 */
bool getLeftMotorVelocity(serial::Serial *s, int16_t *velocity)
{
    std::vector<uint8_t> outData;

    if (!executeCommand(s, 0x9A, {0x00}, 5, &outData))
        return false;

    if (outData.size() < 2)
    {
        std::cerr << "Invalid response length." << std::endl;
        return false;
    }

    *velocity = (static_cast<int16_t>(outData[0]) << 8) | outData[1];
    return true;
}

/**
 * @brief Sets the velocity in range of 512 to -512 depending on range of allowed velocity for safety on/off
 *
 * Set right motor velocity (raw, two bytes); expected response length: 3 bytes.
 *
 * Note: Velocity = 512-Full forward,
 * = 0-Stop,
 * = -512-Full reverse
 *
 * Robot direction command will act exactly opposite if motor velocity is set to -ve value.
 * e.g if velocity set for motor is -ve, then sending forward direction command will move the
 * motor in reverse direction.
 *
 * Any change in velocity of robot will reflect only and only after resending robot
 * direction command.
 *
 * @param s
 * @param velocity
 * @return true
 * @return false
 */
bool setRightMotorVelocity(serial::Serial *s, int16_t *velocity)
{
    uint8_t msb = static_cast<uint8_t>(*velocity >> 8);
    uint8_t lsb = static_cast<uint8_t>(*velocity & 0xFF);
    return executeCommand(s, 0x96, {msb, lsb}, 3);
}

/**
 * @brief Get the Right Motor Velocity object
 *
 * Returns the velocity in range of 512 to -512 depending on range of allowed velocity for safety on/off
 *
 * Get right motor velocity (raw); expected response length: 5 bytes.
 *
 * @param s
 * @param velocity
 * @return true
 * @return false
 */
bool getRightMotorVelocity(serial::Serial *s, int16_t *velocity)
{
    std::vector<uint8_t> outData;

    if (!executeCommand(s, 0x9B, {0x00}, 5, &outData))
        return false;

    if (outData.size() < 2)
    {
        std::cerr << "Invalid response length." << std::endl;
        return false;
    }

    *velocity = (static_cast<int16_t>(outData[0]) << 8) | outData[1];
    return true;
}

/**
 * @brief Set left motor velocity in m/s; expected response length: 3 bytes.
 * 
 * @param s 
 * @param velocity 
 * @return true 
 * @return false 
 */
bool setLeftMotorVelocity_mps(serial::Serial *s, int16_t *velocity)
{

    *velocity = (*velocity) * 1000; // converting m/s to mm/s to satisfy protocol
    uint8_t msb = static_cast<uint8_t>(*velocity >> 8);
    uint8_t lsb = static_cast<uint8_t>(*velocity & 0xFF);
    return executeCommand(s, 0x70, {msb, lsb}, 3);
}

/**
 * @brief Get left motor velocity in m/s; expected response length: 5 bytes.
 * 
 * @param s 
 * @param outData 
 * @return true 
 * @return false 
 */
bool getLeftMotorVelocity_mmps(serial::Serial *s, int16_t *velocity)
{
    std::vector<uint8_t> outData;
    if(!executeCommand(s, 0x76, {0x00}, 5, &outData))
        return false;
    
    *velocity = (static_cast<int16_t>(outData[0]) << 8) | outData[1];
    *velocity /=1000;
    return true;
}

// 3.3.14 Set right motor velocity in m/s (two bytes); expected response length: 3 bytes.
bool setRightMotorVelocity_mps(serial::Serial *s, uint8_t msb, uint8_t lsb)
{
    return executeCommand(s, 0x71, {msb, lsb}, 3);
}

// 3.3.15 Get right motor velocity in mm/s; expected response length: 5 bytes.
bool getRightMotorVelocity_mmps(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x77, {0x00}, 5, outData);
}

// 3.3.16 Set left motor velocity in rad/s (two bytes); expected response length: 3 bytes.
bool setLeftMotorVelocity_radps(serial::Serial *s, uint8_t msb, uint8_t lsb)
{
    return executeCommand(s, 0x7B, {msb, lsb}, 3);
}

// 3.3.17 Get left motor velocity in rad/s; expected response length: 5 bytes.
bool getLeftMotorVelocity_radps(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x7D, {0x00}, 5, outData);
}

// 3.3.18 Set right motor velocity in rad/s (two bytes); expected response length: 3 bytes.
bool setRightMotorVelocity_radps(serial::Serial *s, uint8_t msb, uint8_t lsb)
{
    return executeCommand(s, 0x7C, {msb, lsb}, 3);
}

// 3.3.19 Get right motor velocity in rad/s; expected response length: 5 bytes.
bool getRightMotorVelocity_radps(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x7E, {0x00}, 5, outData);
}

// 3.3.20 Set robot angular velocity in rad/s (two bytes); expected response length: 3 bytes.
bool setRobotAngularVelocity(serial::Serial *s, uint8_t msb, uint8_t lsb)
{
    return executeCommand(s, 0x74, {msb, lsb}, 3);
}

// 3.3.21 Get robot angular velocity in rad/s; expected response length: 5 bytes.
bool getRobotAngularVelocity(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x78, {0x00}, 5, outData);
}

// 3.3.22 Set robot direction (0x01: Forward, 0x02: Reverse, 0x03: Left, 0x04: Right, 0x06: Stop); expected response length: 3 bytes.
bool setRobotDirection(serial::Serial *s, uint8_t direction)
{
    return executeCommand(s, 0x94, {direction}, 3);
}

// 3.3.23 Get left motor encoder counts (assumed command 0x92); expected response length: 5 bytes.
bool getLeftMotorEncoderCounts(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x92, {}, 5, outData);
}

// 3.3.24 Get right motor encoder counts (assumed command 0x93); expected response length: 5 bytes.
bool getRightMotorEncoderCounts(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x93, {}, 5, outData);
}

// 3.3.25 Clear encoder counts (assumed command 0x9C); expected response length: 3 bytes.
bool clearEncoderCounts(serial::Serial *s)
{
    return executeCommand(s, 0x9C, {}, 3);
}

// 3.3.26 Set position: Command=0x40 with 4 bytes left pos, 1 byte left vel, 4 bytes right pos, 1 byte right vel; expected response length: 3 bytes.
bool setPosition(serial::Serial *s, uint32_t posLeft, uint8_t velLeft, uint32_t posRight, uint8_t velRight)
{
    std::vector<uint8_t> data;
    data.push_back((posLeft >> 24) & 0xFF);
    data.push_back((posLeft >> 16) & 0xFF);
    data.push_back((posLeft >> 8) & 0xFF);
    data.push_back(posLeft & 0xFF);
    data.push_back(velLeft);
    data.push_back((posRight >> 24) & 0xFF);
    data.push_back((posRight >> 16) & 0xFF);
    data.push_back((posRight >> 8) & 0xFF);
    data.push_back(posRight & 0xFF);
    data.push_back(velRight);
    return executeCommand(s, 0x40, data, 3);
}

// 3.3.27 Set linear position (assumed command 0x41, 4 bytes); expected response length: 3 bytes.
bool setLinearPosition(serial::Serial *s, uint32_t position)
{
    std::vector<uint8_t> data = {
        uint8_t((position >> 24) & 0xFF),
        uint8_t((position >> 16) & 0xFF),
        uint8_t((position >> 8) & 0xFF),
        uint8_t(position & 0xFF)};
    return executeCommand(s, 0x41, data, 3);
}

// 3.3.28 Set angular position (assumed command 0x42, 4 bytes); expected response length: 3 bytes.
bool setAngularPosition(serial::Serial *s, uint32_t position)
{
    std::vector<uint8_t> data = {
        uint8_t((position >> 24) & 0xFF),
        uint8_t((position >> 16) & 0xFF),
        uint8_t((position >> 8) & 0xFF),
        uint8_t(position & 0xFF)};
    return executeCommand(s, 0x42, data, 3);
}

// 3.3.29 Get left motor voltage (assumed command 0x9D); expected response length: 4 bytes.
bool getLeftMotorVoltage(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x9D, {}, 4, outData);
}

// 3.3.30 Get right motor voltage (assumed command 0x9E); expected response length: 4 bytes.
bool getRightMotorVoltage(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x9E, {}, 4, outData);
}

// 3.3.31 Get left motor current (assumed command 0x9F); expected response length: 4 bytes.
bool getLeftMotorCurrent(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x9F, {}, 4, outData);
}

// 3.3.32 Get right motor current (assumed command 0xA0); expected response length: 4 bytes.
bool getRightMotorCurrent(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0xA0, {}, 4, outData);
}

// 3.3.33 Set wheel diameter in micron: Command=0x50, two bytes; expected response length: 3 bytes.
bool setWheelDiameterMicron(serial::Serial *s, uint16_t diameter)
{
    return executeCommand(s, 0x50, {uint8_t(diameter >> 8), uint8_t(diameter & 0xFF)}, 3);
}

// 3.3.34 Get wheel diameter in micron: Command=0x51; expected response length: 4 bytes.
bool getWheelDiameterMicron(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x51, {0x00}, 4, outData);
}

// 3.3.35 Set axle length in micron: Command=0x52, two bytes; expected response length: 3 bytes.
bool setAxleLengthMicron(serial::Serial *s, uint16_t length)
{
    return executeCommand(s, 0x52, {uint8_t(length >> 8), uint8_t(length & 0xFF)}, 3);
}

// 3.3.36 Get axle length in micron: Command=0x53; expected response length: 4 bytes.
bool getAxleLengthMicron(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x53, {0x00}, 4, outData);
}

// 3.3.37 Set maximum robot velocity in m/s (assumed command 0x54, two bytes); expected response length: 3 bytes.
bool setMaxRobotVelocity_mps(serial::Serial *s, uint16_t vel)
{
    return executeCommand(s, 0x54, {uint8_t(vel >> 8), uint8_t(vel & 0xFF)}, 3);
}

// 3.3.38 Get maximum robot velocity in mm/s (assumed command 0x55, sub=0x00); expected response length: 4 bytes.
bool getMaxRobotVelocity_mmps(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x55, {0x00}, 4, outData);
}

// 3.3.39 Get position encoder resolution (assumed command 0x56, sub=0x00); expected response length: 4 bytes.
bool getEncoderResolution(serial::Serial *s, std::vector<uint8_t> *outData)
{
    return executeCommand(s, 0x56, {0x00}, 4, outData);
}

#endif //__MOTOR_CONTORL_APIS__