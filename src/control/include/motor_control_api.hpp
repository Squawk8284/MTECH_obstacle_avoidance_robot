/**
 * @file motor_control_api.hpp
 * @author Kartik Sahasrabudhe
 * @brief Motor control API for NEX robot using vector‐based commands.
 * @version 0.3
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 */
#ifndef __MOTOR_CONTORL_APIS_H__
#define __MOTOR_CONTORL_APIS_H__

#include <utils.hpp>

// Convenience macros for safety and mode values.
#define SAFETY_ON (1)
#define SAFETY_OFF (0)

// Robot Mode
#define OPEN_LOOP_CONTROL (0)
#define CLOSED_LOOP_CONTROL (1)
#define POSITION_CONTROL (2)

// Robot Direction
#define FORWARD (0x01)
#define REVERSE (0x02)
#define LEFT (0x03)
#define RIGHT (0x04)
#define STOP (0x06)

// ---------------------------------------------------------------------------
// Motor Control Commands
// ---------------------------------------------------------------------------

/**
 * @brief Set safety timeout (in seconds); expected response length: 3 bytes.
 *
 * If no command is sent to the robot before the timeout, it will simply stop its motion.
 *
 * @param s Pointer to the serial object.
 * @param timeout Timeout value in seconds. (Integer only)
 * @return true if the command is successfully set.
 * @return false if the command fails to execute.
 */
bool setSafetyTimeout(serial::Serial *s, float timeout)
{
    _DEBUG(PrintLn("Setting the Safety Timeout to ", static_cast<int8_t>(timeout), "s"));
    uint8_t timeoutByte = static_cast<uint8_t>(timeout);
    if (!executeCommand(s, CMD(SetSafetyTimeout, 0x7A, 0x01), &timeoutByte, sizeof(timeoutByte), ReturnPayload(0), nullptr, 0))
    {
        return FAILURE;
    }
    return SUCCESS;
}

/**
 * @brief Get safety timeout; expected response length: 4 bytes.
 *
 * The returned payload (after skipping header, command and checksum) is 1 byte.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector in which to store the response payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getSafetyTimeout(serial::Serial *s, float *safetyTimeout)
{
    uint8_t buffer = 0;
    if (!executeCommand(s, CMD(GetSafetyTimeout, 0x7A, 0x02), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    *safetyTimeout = static_cast<float>(buffer);

    return SUCCESS;
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
    if (!executeCommand(s, CMD(SetSafetyMode, 0x89), &mode, sizeof(mode), ReturnPayload(0), nullptr, 0))
    {
        return FAILURE;
    }
    return SUCCESS;
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
    if (!executeCommand(s, CMD(SetRobotMode, 0x90), &mode, sizeof(mode), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
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
bool getRobotMode(serial::Serial *s, uint8_t *RobotMode)
{
    if (!executeCommand(s, CMD(GetRobotMode, 0x91, 0x00), nullptr, 0, ReturnPayload(1), RobotMode, sizeof(RobotMode)))
        return FAILURE;
    return SUCCESS;
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
    if (!executeCommand(s, CMD(SetAcceleration, 0x98), &acc, sizeof(acc), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
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
bool getAcceleration(serial::Serial *s, float *Acceleration)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(GetAcceleration, 0x99, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    *Acceleration = buffer * 168.72; // As per the software Manual
    return SUCCESS;
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
    data[0] = static_cast<uint8_t>((*velocity) >> 8);   // MSB
    data[1] = static_cast<uint8_t>((*velocity) & 0xFF); // LSB
    if (!executeCommand(s, CMD(SetLeftMotorVelocity, 0x95), data, sizeof(data), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
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
    uint8_t buffer[2] = {0}; // expected payload: 2 bytes
    if (!executeCommand(s, CMD(GetLeftMotorVelocity, 0x9A, 0x00), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;
    *velocity = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1]; // Re-arranging the MSB and LSB
    return SUCCESS;
}

/**
 * @brief Set right motor velocity (raw, two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to an int16_t holding the desired velocity.
 * @return true if successful.
 * @return false if failed.
 */
bool setRightMotorVelocity(serial::Serial *s, int16_t velocity)
{
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(velocity >> 8);   // MSB
    buffer[1] = static_cast<uint8_t>(velocity & 0xFF); // LSB
    if (!executeCommand(s, CMD(SetRightMotorVelocity, 0x96), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
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
    uint8_t buffer[2] = {0};
    if (!executeCommand(s, CMD(GetRightMotorVelocity, 0x9B, 0x00), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;
    *velocity = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1]; // Re-arranging the MSB and LSB
    return SUCCESS;
}

/**
 * @brief Set left motor velocity in m/s; expected response length: 3 bytes.
 *
 * Converts m/s to mm/s (by multiplying by 1000) to satisfy the protocol.
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to a float containing the velocity in m/s.
 * @return true if successful.
 * @return false if failed.
 */
bool setLeftMotorVelocity_mps(serial::Serial *s, float velocity)
{
    int16_t vel_mm = static_cast<int16_t>((velocity) * 1000);
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(vel_mm >> 8);
    buffer[1] = static_cast<uint8_t>(vel_mm & 0xFF);
    if (!executeCommand(s, CMD(SetLeftMotorVelocity_mps, 0x70), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Get left motor velocity in m/s; expected response length: 5 bytes.
 *
 * Converts the raw mm/s value to m/s (by dividing by 1000).
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to a float where the m/s value will be stored.
 * @return true if successful.
 * @return false if failed.
 */
bool getLeftMotorVelocity_mps(serial::Serial *s, float *velocity)
{
    uint8_t buffer[2] = {0};
    if (!executeCommand(s, CMD(GetLeftMotorVelocity_mps, 0x76, 0x00), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;
    int16_t vel_mm = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    *velocity = vel_mm / 1000.0f;
    return SUCCESS;
}

/**
 * @brief Set right motor velocity in m/s (two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param velocity Pointer to a float containing the velocity in m/s.
 * @return true if successful.
 * @return false if failed.
 */
bool setRightMotorVelocity_mps(serial::Serial *s, float velocity)
{
    int16_t vel_mm = static_cast<int16_t>((velocity) * 1000);
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(vel_mm >> 8);
    buffer[1] = static_cast<uint8_t>(vel_mm & 0xFF);
    if (!executeCommand(s, CMD(SetRightMotorVelocity_mps, 0x71), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Get right motor velocity in m/s; expected response length: 5 bytes.
 *
 * Returns the raw payload (2 bytes) in a vector.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRightMotorVelocity_mps(serial::Serial *s, float *velocity)
{
    uint8_t buffer[2] = {0};
    if (!executeCommand(s, CMD(GetRightMotorVelocity_mps, 0x77, 0x00), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;
    int16_t vel_mm = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    *velocity = vel_mm / 1000.0f;
    return SUCCESS;
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
bool setLeftMotorVelocity_radps(serial::Serial *s, float *velocity_radps)
{
    int16_t vel_rad = static_cast<int16_t>((*velocity_radps) * 1000);
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(vel_rad >> 8);
    buffer[1] = static_cast<uint8_t>(vel_rad & 0xFF);
    if (!executeCommand(s, CMD(SetLeftMotorVelocity_radps, 0x7B), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Get left motor velocity in rad/s; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getLeftMotorVelocity_radps(serial::Serial *s, float *velocity_radps)
{
    uint8_t buffer[2] = {0};
    if (!executeCommand(s, CMD(GetLeftMotorVelocity_radps, 0x7D, 0x00), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;
    int16_t vel_rad = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    *velocity_radps = vel_rad / 1000.0f;
    return SUCCESS;
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
bool setRightMotorVelocity_radps(serial::Serial *s, float *velocity_radps)
{
    int16_t vel_rad = static_cast<int16_t>((*velocity_radps) * 1000);
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(vel_rad >> 8);
    buffer[1] = static_cast<uint8_t>(vel_rad & 0xFF);
    if (!executeCommand(s, CMD(SetRightMotorVelocity_radps, 0x7C), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Get right motor velocity in rad/s; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRightMotorVelocity_radps(serial::Serial *s, float *velocity_radps)
{

    uint8_t buffer[2] = {0};
    if (!executeCommand(s, CMD(GetRightMotorVelocity_radps, 0x7E, 0x00), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;
    int16_t vel_rad = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    *velocity_radps = vel_rad / 1000.0f;
    return SUCCESS;
}

/**
 * @brief Set robot angular velocity in rad/s (two bytes); expected response length: 3 bytes.
 * Anticlockwise velocity is positive
 *
 * @param s Pointer to the serial object.
 * @param msb Most significant byte.
 * @param lsb Least significant byte.
 * @return true if successful.
 * @return false if failed.
 */
bool setRobotAngularVelocityRadps(serial::Serial *s, float velocity_angular)
{
    int16_t vel_ang = static_cast<int16_t>((velocity_angular) * 1000);
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(vel_ang >> 8);
    buffer[1] = static_cast<uint8_t>(vel_ang & 0xFF);
    if (!executeCommand(s, CMD(SetRobotAngularVelocityRadps, 0x74), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Get robot angular velocity in rad/s; expected response length: 5 bytes.
 * Anticlockwise velocity is positive
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRobotAngularVelocityRadps(serial::Serial *s, float *velocity_angular)
{
    uint8_t buffer[2] = {0};
    if (!executeCommand(s, CMD(GetRobotAngularVelocityRadps, 0x78, 0x00), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;
    int16_t vel_ang = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    *velocity_angular = vel_ang / 1000.0f;
    return SUCCESS;
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
    if (!executeCommand(s, CMD(SetRobotDirection, 0x94), &direction, sizeof(direction), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Get left motor encoder counts; assumed command 0x92; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getLeftMotorEncoderCounts(serial::Serial *s, uint32_t *LeftEncoderCounts)
{
    uint8_t buffer[4] = {0};
    if (!executeCommand(s, CMD(GetLeftMotorEncoderCounts, 0x92, 0x00), nullptr, 0, ReturnPayload(4), buffer, sizeof(buffer)))
        return FAILURE;

    *LeftEncoderCounts = (static_cast<uint32_t>(buffer[0]) << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3]);
    return SUCCESS;
}

/**
 * @brief Get right motor encoder counts; assumed command 0x93; expected response length: 5 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getRightMotorEncoderCounts(serial::Serial *s, uint32_t *RightEncoderCounts)
{
    uint8_t buffer[4] = {0};
    if (!executeCommand(s, CMD(GetRightMotorEncoderCounts, 0x93, 0x00), nullptr, 0, ReturnPayload(4), buffer, sizeof(buffer)))
        return FAILURE;

    *RightEncoderCounts = (static_cast<uint32_t>(buffer[0]) << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3]);
    return SUCCESS;
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
    if (!executeCommand(s, CMD(ClearEncoderCounts, 0x8C, 0x00), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Set the Position object by encoder counts
 * For desired behavior of robot, before sending following command user should select
 * “position control mode”.
 *
 * Velocity = 512 - Full forward,
 * = 0 - Stop,
 * = -512 - Full reverse
 *
 * Every new Set Position command will reset the encoder count registers of both the
 * motors and the target encoder count will be achieved by starting from zero.
 * @param s Pointer to the serial object.
 * @param leftEncoderCounts Left Encoder Counts
 * @param velLeft   Left velocity
 * @param rightEncoderCounts    Right Encoder Counts
 * @param velRight  Right Velocity
 * @return true if successful.
 * @return false if failed.
 */
bool setPositionbyEncoderCounts(serial::Serial *s, uint32_t leftEncoderCounts, int16_t velLeft, uint32_t rightEncoderCounts, int16_t velRight)
{
    uint8_t buffer[12]; // Use uint8_t for byte-wise storage

    // Pack left encoder counts (32-bit)
    buffer[0] = static_cast<uint8_t>((leftEncoderCounts >> 24) & 0xFF);
    buffer[1] = static_cast<uint8_t>((leftEncoderCounts >> 16) & 0xFF);
    buffer[2] = static_cast<uint8_t>((leftEncoderCounts >> 8) & 0xFF);
    buffer[3] = static_cast<uint8_t>(leftEncoderCounts & 0xFF);

    // Pack left velocity (16-bit)
    buffer[4] = static_cast<uint8_t>((velLeft >> 8) & 0xFF);
    buffer[5] = static_cast<uint8_t>(velLeft & 0xFF);

    // Pack right encoder counts (32-bit)
    buffer[6] = static_cast<uint8_t>((rightEncoderCounts >> 24) & 0xFF);
    buffer[7] = static_cast<uint8_t>((rightEncoderCounts >> 16) & 0xFF);
    buffer[8] = static_cast<uint8_t>((rightEncoderCounts >> 8) & 0xFF);
    buffer[9] = static_cast<uint8_t>(rightEncoderCounts & 0xFF);

    // Pack right velocity (16-bit)
    buffer[10] = static_cast<uint8_t>((velRight >> 8) & 0xFF);
    buffer[11] = static_cast<uint8_t>(velRight & 0xFF);

    // Send command
    if (!executeCommand(s, CMD(SetPositionByEncoderCounts, 0x9C), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Set the Positionby Distance In Meters object
 * For desired behavior of robot, before sending following command user should select
 * “position control mode”.
 *
 * Every new Set linear position command will reset the encoder count registers of both
 * the motors and the target encoder count will be achieved by starting from zero.
 *
 * @param s Pointer to the serial object.
 * @param leftMotorDistance Distance in m
 * @param velLeft   Velocity in m/s
 * @param rightMotorDistance    Distance in m
 * @param velRight  Velocity in m/s
 * @return true if successful
 * @return false if failed
 */
bool setPositionbyDistanceInMeters(serial::Serial *s, float leftMotorDistance, float velLeft, float rightMotorDistance, float velRight)
{
    // Convert distances and velocities to mm
    int32_t leftDist = static_cast<int32_t>(leftMotorDistance * 1000);
    int16_t leftVel = static_cast<int16_t>(velLeft * 1000);
    int32_t rightDist = static_cast<int32_t>(rightMotorDistance * 1000);
    int16_t rightVel = static_cast<int16_t>(velRight * 1000);

    uint8_t buffer[12]; // Use uint8_t for byte-wise storage

    // Pack left motor distance (32-bit)
    buffer[0] = static_cast<uint8_t>((leftDist >> 24) & 0xFF);
    buffer[1] = static_cast<uint8_t>((leftDist >> 16) & 0xFF);
    buffer[2] = static_cast<uint8_t>((leftDist >> 8) & 0xFF);
    buffer[3] = static_cast<uint8_t>(leftDist & 0xFF);

    // Pack left motor velocity (16-bit)
    buffer[4] = static_cast<uint8_t>((leftVel >> 8) & 0xFF);
    buffer[5] = static_cast<uint8_t>(leftVel & 0xFF);

    // Pack right motor distance (32-bit)
    buffer[6] = static_cast<uint8_t>((rightDist >> 24) & 0xFF);
    buffer[7] = static_cast<uint8_t>((rightDist >> 16) & 0xFF);
    buffer[8] = static_cast<uint8_t>((rightDist >> 8) & 0xFF);
    buffer[9] = static_cast<uint8_t>(rightDist & 0xFF);

    // Pack right motor velocity (16-bit)
    buffer[10] = static_cast<uint8_t>((rightVel >> 8) & 0xFF);
    buffer[11] = static_cast<uint8_t>(rightVel & 0xFF);

    // Send command
    if (!executeCommand(s, CMD(SetPosition, 0x72), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE; // Use a defined failure macro if applicable

    return SUCCESS; // Use a defined success macro if applicable
}

/**
 * @brief Set the Angular Position
 *
 * For desired behavior of robot, before sending following command user should select
 * “position control mode”.
 *
 * Every new Set angular position command will reset encoder count registers of both the
 * motors and the target encoder count will be achieved by starting from zero.
 *
 * @param s Pointer to the serial object.
 * @param angle Angle to move to
 * @param velocity_radps rad/s would also decide the direction of turn. (anticlockwise -- positive)
 * @return true if successfull
 * @return false if failed
 */
bool setAngularPosition(serial::Serial *s, float angle, float velocity_radps)
{
    // Convert to integer values as per software manual
    int32_t angle_int = static_cast<int32_t>(angle * 1000);             // 4-byte integer
    int16_t velocity_int = static_cast<int16_t>(velocity_radps * 1000); // 2-byte integer

    uint8_t buffer[6]; // Use uint8_t for byte-wise storage

    // Pack angle (32-bit)
    buffer[0] = static_cast<uint8_t>((angle_int >> 24) & 0xFF);
    buffer[1] = static_cast<uint8_t>((angle_int >> 16) & 0xFF);
    buffer[2] = static_cast<uint8_t>((angle_int >> 8) & 0xFF);
    buffer[3] = static_cast<uint8_t>(angle_int & 0xFF);

    // Pack velocity (16-bit)
    buffer[4] = static_cast<uint8_t>((velocity_int >> 8) & 0xFF);
    buffer[5] = static_cast<uint8_t>(velocity_int & 0xFF);

    // Send command
    return executeCommand(s, CMD(SetAngularPosition, 0x42), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0);
}

/**
 * @brief Get the Left Motor Voltage
 *
 * @param s Pointer to the serial object.
 * @param leftMotorVoltage Left Motor Voltage
 * @return true if successfull
 * @return false if failed
 */
bool getLeftMotorVoltage(serial::Serial *s, float *leftMotorVoltage)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(GetLeftMotorVoltage, 0x97, 0x03), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *leftMotorVoltage = static_cast<float>(buffer);
    return SUCCESS;
}

/**
 * @brief Get the Right Motor Voltage
 *
 * @param s Pointer to the serial object.
 * @param rightMotorVoltage Right Motor Voltage
 * @return true if successfull
 * @return false if failed
 */
bool getRightMotorVoltage(serial::Serial *s, float *rightMotorVoltage)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(GetRightMotorVoltage, 0x97, 0x04), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *rightMotorVoltage = static_cast<float>(buffer);
    return SUCCESS;
}

/**
 * @brief Get the Left Motor Current
 *
 * @param s Pointer to the serial object.
 * @param leftMotorCurrent Left Motor Current
 * @return true if successfull
 * @return false if failed
 */
bool getLeftMotorCurrent(serial::Serial *s, float *leftMotorCurrent)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(GetLeftMotorCurrent, 0x97, 0x01), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *leftMotorCurrent = static_cast<float>(buffer);
    return SUCCESS;
}

/**
 * @brief Get the Right Motor Current
 *
 * @param s Pointer to the serial object.
 * @param rightMotorCurrent Right Motor Current
 * @return true if successfull
 * @return false if failed
 */
bool getRightMotorCurrent(serial::Serial *s, float *rightMotorCurrent)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(GetRightMotorCurrent, 0x97, 0x02), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *rightMotorCurrent = static_cast<float>(buffer);
    return SUCCESS;
}

/**
 * @brief Set wheel diameter in mm
 *
 * @param s Pointer to the serial object.
 * @param diameter Wheel diameter in mm.
 * @return true if successful.
 * @return false if failed.
 */
bool setWheelDiameter_mm(serial::Serial *s, float diameter)
{
    // Convert to microns (1 mm = 1000 microns)
    uint32_t diameter_microns = static_cast<uint32_t>(diameter * 1000);

    uint8_t buffer[4];

    // Correct way to extract bytes
    buffer[0] = static_cast<uint8_t>((diameter_microns >> 24) & 0xFF);
    buffer[1] = static_cast<uint8_t>((diameter_microns >> 16) & 0xFF);
    buffer[2] = static_cast<uint8_t>((diameter_microns >> 8) & 0xFF);
    buffer[3] = static_cast<uint8_t>(diameter_microns & 0xFF);

    // Execute the command
    if (!executeCommand(s, CMD(SetWheelDiameterMicron, 0x79, 0x01), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Get the WheelDiameter m object
 *
 * @param s Pointer to the serial object.
 * @param diameter Wheel diameter in m.
 * @return true if successful.
 * @return false if failed.
 */
bool getWheelDiameter_m(serial::Serial *s, float *diameter)
{
    uint8_t buffer[4];
    if (!executeCommand(s, CMD(GetWheelDiameter, 0x79, 0x02), nullptr, 0, ReturnPayload(4), buffer, sizeof(buffer)))
        return FAILURE;

    uint32_t raw_diamter = (static_cast<uint32_t>(buffer[0]) << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3]);
    *diameter = raw_diamter / 1000000.0f;
    return SUCCESS;
}

/**
 * @brief Set the AxleLength mm
 *
 * @param s Pointer to the serial object.
 * @param length Axel length in mm
 * @return true if successful.
 * @return false if failed.
 */
bool setAxleLength_mm(serial::Serial *s, float length)
{
    // Convert to integer (in microns)
    uint32_t length_microns = static_cast<uint32_t>(length * 1000);

    uint8_t buffer[4];

    // Properly pack into bytes
    buffer[0] = static_cast<uint8_t>((length_microns >> 24) & 0xFF);
    buffer[1] = static_cast<uint8_t>((length_microns >> 16) & 0xFF);
    buffer[2] = static_cast<uint8_t>((length_microns >> 8) & 0xFF);
    buffer[3] = static_cast<uint8_t>(length_microns & 0xFF);

    // Send command
    if (!executeCommand(s, CMD(SetAxelLength, 0x79, 0x03), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Get the Axle Length m
 *
 * @param s Pointer to the serial object.
 * @param length Axel length in m
 * @return true if successful.
 * @return false if failed.
 */
bool getAxleLength_m(serial::Serial *s, float *length)
{
    uint8_t buffer[4];
    if (!executeCommand(s, CMD(GetAxelLength, 0x79, 0x04), nullptr, 0, ReturnPayload(4), buffer, sizeof(buffer)))
        return FAILURE;

    uint32_t raw_length = (static_cast<uint32_t>(buffer[0]) << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3]);
    *length = raw_length / 1000000.0f;
    return SUCCESS;
}

/**
 * @brief Set maximum robot velocity in m/s; Command 0x54 (two bytes); expected response length: 3 bytes.
 *
 * @param s Pointer to the serial object.
 * @param vel Maximum velocity in m/s (converted to raw value as needed).
 * @return true if successful.
 * @return false if failed.
 */
bool setMaxRobotVelocity_mps(serial::Serial *s, float vel)
{
    // Convert velocity to mm/s and store as signed 16-bit integer
    int16_t velocity_mmps = static_cast<int16_t>(vel * 1000);

    uint8_t buffer[2];

    // Properly pack velocity as 16-bit signed integer
    buffer[0] = static_cast<uint8_t>((velocity_mmps >> 8) & 0xFF);
    buffer[1] = static_cast<uint8_t>(velocity_mmps & 0xFF);

    // Send command
    if (!executeCommand(s, CMD(SetMaxRobotVelocity, 0x79, 0x05), buffer, sizeof(buffer), ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Get maximum robot velocity in mm/s; Command 0x55; expected response length: 4 bytes.
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getMaxRobotVelocity_mps(serial::Serial *s, float *velocity)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(GetMaxRobotVelocity_mps, 0x79, 0x06), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_velocity = (static_cast<int16_t>(buffer[0]) << 8) | (buffer[1]);
    *velocity = raw_velocity / 1000.0f;
    return SUCCESS;
}

/**
 * @brief Get position encoder resolution in counts/WheelRevolution
 *
 * @param s Pointer to the serial object.
 * @param outData Pointer to a vector to store the payload.
 * @return true if successful.
 * @return false if failed.
 */
bool getEncoderResolutionCountPerWheelRevolution(serial::Serial *s, uint16_t *EncoderResolutionCount)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(GetEncoderResolutionCountPerWheelRevolution, 0x79, 0x07), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;
    *EncoderResolutionCount = (static_cast<uint16_t>(buffer[0]) << 8) | (buffer[1]);
    return SUCCESS;
}

#endif // __MOTOR_CONTORL_APIS_H__
