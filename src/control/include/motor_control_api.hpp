/**
 * @file motor_control_api.hpp
 * @author Kartik Sahasrabudhe
 * @brief Motor control API for NEX robot.
 *        This file converts function arguments and responses to/from raw bytes using the new
 *        void*-based interface in nex_robot.hpp.
 * @version 0.1
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 */
#ifndef __MOTOR_CONTORL_APIS__
#define __MOTOR_CONTORL_APIS__

#include <nex_robot.hpp>
#include <vector>
#include <cstdint>
#include <cstring>  // For memcpy

#define SAFETY_ON  (1)
#define SAFETY_OFF (0)

#define OPEN_LOOP_CONTROL   (0)
#define CLOSED_LOOP_CONTROL (1)
#define POSITION_CONTROL    (2)

// ---------------------------------------------------------------------------
// Motor Control Commands
// ---------------------------------------------------------------------------

/**
 * @brief Set safety timeout (in seconds); expected response length: 3 bytes.
 *
 * If no command is sent to the robot before the timeout, it will simply stop its motion.
 *
 * @param s Pointer to the serial object.
 * @param timeout Timeout value in seconds (int8).
 * @return true if the command is successfully set.
 * @return false if the command fails to execute.
 */
bool setSafetyTimeout(serial::Serial *s, float timeout)
{
    std::cout << "Setting Safety Timeout to " << timeout << "s" << std::endl;
    uint8_t data[2] = { 0x01, static_cast<uint8_t>(timeout) };
    // expected response length = 3 bytes; no output data needed.
    return executeCommand(s, 0x7A, data, sizeof(data), 3, nullptr, 0);
}


/**
 * @brief Get safety timeout; expected response length: 4 bytes.
 *
 * The returned payload (after skipping header and command) is 2 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector in which to store the response payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getSafetyTimeout(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t data = 0x02;
    uint8_t buffer = 0; // 4 bytes expected response -> payload = 2 bytes
    bool success = executeCommand(s, 0x7A, &data, sizeof(data), 4, &buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(sizeof(buffer), buffer);
    }
    return success;
}

/**
 * @brief Set safety mode (1 for ON, 0 for OFF); expected response length: 3 bytes.
 *
 * During safety ON, the robot's maximum velocity is limited.
 *
 * @param s Pointer to the serial object.
 * @param mode Safety mode (1 for ON, 0 for OFF).
 * @return true if successful.
 * @return false if failed.
 */
bool setSafetyMode(serial::Serial *s, uint8_t mode)
{
    return executeCommand(s, 0x89, &mode, sizeof(mode), 3, nullptr, 0);
}

/**
 * @brief Set robot mode (0: open-loop, 1: closed-loop, 2: position); expected response length: 3 bytes.
 *
 * In position control mode, every new position command resets the encoder count.
 *
 * @param s Pointer to the serial object.
 * @param mode Robot mode.
 * @return true if successful.
 * @return false if failed.
 */
bool setRobotMode(serial::Serial *s, uint8_t mode)
{
    return executeCommand(s, 0x90, &mode, sizeof(mode), 3, nullptr, 0);
}

/**
 * @brief Get robot mode; expected response length: 4 bytes.
 *
 * Payload values: 0 = Open loop, 1 = Closed loop, 2 = Position.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the 2-byte payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRobotMode(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t data = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x91, &data, sizeof(data), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Set acceleration (1-16); expected response length: 3 bytes.
 *
 * Acceleration = (168.72 * n) mm/sec².
 *
 * @param s Pointer to the serial object.
 * @param acc Acceleration count (1-16).
 * @return true if successful.
 * @return false if failed.
 */
bool setAcceleration(serial::Serial *s, uint8_t acc)
{
    return executeCommand(s, 0x98, &acc, sizeof(acc), 3, nullptr, 0);
}

/**
 * @brief Get acceleration; expected response length: 4 bytes.
 *
 * Returns acceleration in mm/sec².
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the 2-byte payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getAcceleration(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t data = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x99, &data, sizeof(data), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Set left motor velocity (raw, two bytes); expected response length: 3 bytes.
 *
 * Velocity value is in raw units: 512 = full forward, 0 = stop, -512 = full reverse.
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to an int16_t holding the desired velocity.
 * @return true if successful.
 * @return false if failed.
 */
bool setLeftMotorVelocity(serial::Serial *s, int16_t *velocity)
{
    uint8_t data[2];
    data[0] = static_cast<uint8_t>((*velocity) >> 8);
    data[1] = static_cast<uint8_t>((*velocity) & 0xFF);
    return executeCommand(s, 0x95, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Get left motor velocity (raw); expected response length: 5 bytes.
 *
 * Returns the velocity (range 512 to -512) in raw units.
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to an int16_t where the result will be stored.
 * @return true if successful.
 * @return false if failed.
 */
bool getLeftMotorVelocity(serial::Serial *s, int16_t *velocity)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[3] = {0}; // expected response: 5 bytes, payload = 3 bytes; we use first 2 bytes of payload.
    bool success = executeCommand(s, 0x9A, &dummy, sizeof(dummy), 5, buffer, 2);
    if (!success)
        return false;
    *velocity = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    return true;
}

/**
 * @brief Set right motor velocity (raw, two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to an int16_t holding the desired velocity.
 * @return true if successful.
 * @return false if failed.
 */
bool setRightMotorVelocity(serial::Serial *s, int16_t *velocity)
{
    uint8_t data[2];
    data[0] = static_cast<uint8_t>((*velocity) >> 8);
    data[1] = static_cast<uint8_t>((*velocity) & 0xFF);
    return executeCommand(s, 0x96, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Get right motor velocity (raw); expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to an int16_t where the result will be stored.
 * @return true if successful.
 * @return false if failed.
 */
bool getRightMotorVelocity(serial::Serial *s, int16_t *velocity)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[3] = {0}; // payload of 2 bytes expected
    bool success = executeCommand(s, 0x9B, &dummy, sizeof(dummy), 5, buffer, 2);
    if (!success)
        return false;
    *velocity = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    return true;
}

/**
 * @brief Set left motor velocity in m/s; expected response length: 3 bytes.
 *
 * Converts m/s to mm/s (by multiplying by 1000) to satisfy the protocol.
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to an int16_t containing the velocity in m/s.
 * @return true if successful.
 * @return false if failed.
 */
bool setLeftMotorVelocity_mps(serial::Serial *s, float *velocity) 
{
    // Dereference the pointer to get the float value, then convert m/s to mm/s
    int16_t vel_mm = static_cast<int16_t>((*velocity) * 1000);
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(vel_mm >> 8);
    data[1] = static_cast<uint8_t>(vel_mm & 0xFF);
    return executeCommand(s, 0x70, data, sizeof(data), 3, nullptr, 0);
}


/**
 * @brief Get left motor velocity in m/s; expected response length: 5 bytes.
 *
 * Converts the raw mm/s value to m/s (by dividing by 1000).
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to an int16_t where the m/s value will be stored.
 * @return true if successful.
 * @return false if failed.
 */
bool getLeftMotorVelocity_mmps(serial::Serial *s, float *velocity)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[3] = {0};
    bool success = executeCommand(s, 0x76, &dummy, sizeof(dummy), 5, buffer, 2);
    if (!success)
        return false;
    int16_t raw = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    *velocity = raw / 1000; // Convert mm/s to m/s
    return true;
}

/**
 * @brief Set right motor velocity in m/s (two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to an int16_t containing the velocity in m/s.
 * @return true if successful.
 * @return false if failed.
 */
bool setRightMotorVelocity_mps(serial::Serial *s, float *velocity) 
{
    // Dereference the pointer, multiply by 1000, then cast to int16_t.
    int16_t vel_mm = static_cast<int16_t>((*velocity) * 1000);
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(vel_mm >> 8);
    data[1] = static_cast<uint8_t>(vel_mm & 0xFF);
    return executeCommand(s, 0x71, data, sizeof(data), 3, nullptr, 0);
}


/**
 * @brief Get right motor velocity in mm/s; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the raw payload (2 bytes).
 * @return true if successful.
 * @return false if failed.
 */
bool getRightMotorVelocity_mmps(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x77, &dummy, sizeof(dummy), 5, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Set left motor velocity in rad/s (two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param msb Most significant byte.
 * @param lsb Least significant byte.
 * @return true if successful.
 * @return false if failed.
 */
bool setLeftMotorVelocity_radps(serial::Serial *s, uint8_t msb, uint8_t lsb)
{
    uint8_t data[2] = {msb, lsb};
    return executeCommand(s, 0x7B, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Get left motor velocity in rad/s; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getLeftMotorVelocity_radps(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2]; // Expected payload size for rad/s response: usually 2 bytes (adjust if needed)
    // For example, assume 2 bytes payload:
    uint8_t buf[2] = {0};
    bool success = executeCommand(s, 0x7D, &dummy, sizeof(dummy), 5, buf, sizeof(buf));
    if (success && outData)
    {
        outData->assign(buf, buf + sizeof(buf));
    }
    return success;
}

/**
 * @brief Set right motor velocity in rad/s (two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param msb Most significant byte.
 * @param lsb Least significant byte.
 * @return true if successful.
 * @return false if failed.
 */
bool setRightMotorVelocity_radps(serial::Serial *s, uint8_t msb, uint8_t lsb)
{
    uint8_t data[2] = {msb, lsb};
    return executeCommand(s, 0x7C, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Get right motor velocity in rad/s; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRightMotorVelocity_radps(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buf[2] = {0};
    bool success = executeCommand(s, 0x7E, &dummy, sizeof(dummy), 5, buf, sizeof(buf));
    if (success && outData)
    {
        outData->assign(buf, buf + sizeof(buf));
    }
    return success;
}

/**
 * @brief Set robot angular velocity in rad/s (two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param msb Most significant byte.
 * @param lsb Least significant byte.
 * @return true if successful.
 * @return false if failed.
 */
bool setRobotAngularVelocity(serial::Serial *s, uint8_t msb, uint8_t lsb)
{
    uint8_t data[2] = {msb, lsb};
    return executeCommand(s, 0x74, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Get robot angular velocity in rad/s; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRobotAngularVelocity(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buf[2] = {0};
    bool success = executeCommand(s, 0x78, &dummy, sizeof(dummy), 5, buf, sizeof(buf));
    if (success && outData)
    {
        outData->assign(buf, buf + sizeof(buf));
    }
    return success;
}

/**
 * @brief Set robot direction.
 *
 * Valid values: 0x01: Forward, 0x02: Reverse, 0x03: Left, 0x04: Right, 0x06: Stop.
 * Expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param direction Direction command.
 * @return true if successful.
 * @return false if failed.
 */
bool setRobotDirection(serial::Serial *s, uint8_t direction)
{
    return executeCommand(s, 0x94, &direction, sizeof(direction), 3, nullptr, 0);
}

/**
 * @brief Get left motor encoder counts; assumed command 0x92; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getLeftMotorEncoderCounts(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buf[3] = {0}; // Adjust payload size if necessary
    bool success = executeCommand(s, 0x92, &dummy, sizeof(dummy), 5, buf, 3);
    if (success && outData)
    {
        outData->assign(buf, buf + 3);
    }
    return success;
}

/**
 * @brief Get right motor encoder counts; assumed command 0x93; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRightMotorEncoderCounts(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buf[3] = {0};
    bool success = executeCommand(s, 0x93, &dummy, sizeof(dummy), 5, buf, 3);
    if (success && outData)
    {
        outData->assign(buf, buf + 3);
    }
    return success;
}

/**
 * @brief Clear encoder counts; assumed command 0x9C; expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @return true if successful.
 * @return false if failed.
 */
bool clearEncoderCounts(serial::Serial *s)
{
    return executeCommand(s, 0x9C, nullptr, 0, 3, nullptr, 0);
}

/**
 * @brief Set position.
 *
 * Command 0x40 with:
 * - 4 bytes left position,
 * - 1 byte left velocity,
 * - 4 bytes right position,
 * - 1 byte right velocity.
 *
 * Expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param posLeft Left position (4 bytes).
 * @param velLeft Left velocity (1 byte).
 * @param posRight Right position (4 bytes).
 * @param velRight Right velocity (1 byte).
 * @return true if successful.
 * @return false if failed.
 */
bool setPosition(serial::Serial *s, uint32_t posLeft, uint8_t velLeft, uint32_t posRight, uint8_t velRight)
{
    uint8_t data[10];
    data[0] = (posLeft >> 24) & 0xFF;
    data[1] = (posLeft >> 16) & 0xFF;
    data[2] = (posLeft >> 8) & 0xFF;
    data[3] = posLeft & 0xFF;
    data[4] = velLeft;
    data[5] = (posRight >> 24) & 0xFF;
    data[6] = (posRight >> 16) & 0xFF;
    data[7] = (posRight >> 8) & 0xFF;
    data[8] = posRight & 0xFF;
    data[9] = velRight;
    return executeCommand(s, 0x40, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Set linear position; assumed command 0x41, 4 bytes; expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param position Linear position.
 * @return true if successful.
 * @return false if failed.
 */
bool setLinearPosition(serial::Serial *s, uint32_t position)
{
    uint8_t data[4];
    data[0] = (position >> 24) & 0xFF;
    data[1] = (position >> 16) & 0xFF;
    data[2] = (position >> 8) & 0xFF;
    data[3] = position & 0xFF;
    return executeCommand(s, 0x41, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Set angular position; assumed command 0x42, 4 bytes; expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param position Angular position.
 * @return true if successful.
 * @return false if failed.
 */
bool setAngularPosition(serial::Serial *s, uint32_t position)
{
    uint8_t data[4];
    data[0] = (position >> 24) & 0xFF;
    data[1] = (position >> 16) & 0xFF;
    data[2] = (position >> 8) & 0xFF;
    data[3] = position & 0xFF;
    return executeCommand(s, 0x42, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Get left motor voltage; assumed command 0x9D; expected response length: 4 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getLeftMotorVoltage(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x9D, &dummy, sizeof(dummy), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Get right motor voltage; assumed command 0x9E; expected response length: 4 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRightMotorVoltage(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x9E, &dummy, sizeof(dummy), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Get left motor current; assumed command 0x9F; expected response length: 4 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getLeftMotorCurrent(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x9F, &dummy, sizeof(dummy), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Get right motor current; assumed command 0xA0; expected response length: 4 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRightMotorCurrent(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0xA0, &dummy, sizeof(dummy), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Set wheel diameter in micron; Command 0x50 (two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param diameter Wheel diameter in micron.
 * @return true if successful.
 * @return false if failed.
 */
bool setWheelDiameterMicron(serial::Serial *s, uint16_t diameter)
{
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(diameter >> 8);
    data[1] = static_cast<uint8_t>(diameter & 0xFF);
    return executeCommand(s, 0x50, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Get wheel diameter in micron; Command 0x51; expected response length: 4 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getWheelDiameterMicron(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x51, &dummy, sizeof(dummy), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Set axle length in micron; Command 0x52 (two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param length Axle length in micron.
 * @return true if successful.
 * @return false if failed.
 */
bool setAxleLengthMicron(serial::Serial *s, uint16_t length)
{
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(length >> 8);
    data[1] = static_cast<uint8_t>(length & 0xFF);
    return executeCommand(s, 0x52, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Get axle length in micron; Command 0x53; expected response length: 4 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getAxleLengthMicron(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x53, &dummy, sizeof(dummy), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Set maximum robot velocity in m/s; Command 0x54 (two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param vel Maximum velocity in m/s (converted to raw value as needed).
 * @return true if successful.
 * @return false if failed.
 */
bool setMaxRobotVelocity_mps(serial::Serial *s, uint16_t vel)
{
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(vel >> 8);
    data[1] = static_cast<uint8_t>(vel & 0xFF);
    return executeCommand(s, 0x54, data, sizeof(data), 3, nullptr, 0);
}

/**
 * @brief Get maximum robot velocity in mm/s; Command 0x55; expected response length: 4 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getMaxRobotVelocity_mmps(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x55, &dummy, sizeof(dummy), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

/**
 * @brief Get position encoder resolution; Command 0x56; expected response length: 4 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getEncoderResolution(serial::Serial *s, std::vector<uint8_t> *outData)
{
    uint8_t dummy = 0x00;
    uint8_t buffer[2] = {0};
    bool success = executeCommand(s, 0x56, &dummy, sizeof(dummy), 4, buffer, sizeof(buffer));
    if (success && outData)
    {
        outData->assign(buffer, buffer + sizeof(buffer));
    }
    return success;
}

#endif // __MOTOR_CONTORL_APIS__
