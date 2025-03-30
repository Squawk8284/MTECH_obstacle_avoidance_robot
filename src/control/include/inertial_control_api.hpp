/**
 * @file inertial_control_api.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief Inertial API for NEX Robot
 * @version 0.1
 * @date 2025-03-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __INERTIAL_CONTROL_APIS__
#define __INERTIAL_CONTROL_APIS__

#include <nex_robot.hpp>

#define BUZZER_ON (0x01)
#define BUZZER_OFF (0x00)
// ---------------------------------------------------------------------------
// Internal IMU Sensor Commands
// ---------------------------------------------------------------------------
bool get3AxisAccelorometer(serial::Serial *s, float *xAccel, float *yAccel, float *zAccel)
{
    return true;
}

bool setBuzzer(serial::Serial *s, uint8_t buzzer)
{
    if (!executeCommand(s, CMD(SetBuzzer, 0x30), &buzzer, sizeof(buzzer), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

// ---------------------------------------------------------------------------
// External IMU Sensor Commands - 9DOF Razor IMU
// ---------------------------------------------------------------------------

/**
 * @brief Get the orientation of the robot (yaw, pitch, roll) from the IMU.
 *
 * This function sets the IMU to output orientation angles in binary format
 * by sending the "#ob" command. It then sends "#f" to request one frame of data,
 * which is expected to be 12 bytes long (3 floats in little-endian order).
 * The angles (in degrees) are extracted into the provided pointers.
 *
 * @param s Pointer to the serial object connected to the IMU.
 * @param yaw Pointer to a float where the yaw (heading) will be stored.
 * @param pitch Pointer to a float where the pitch will be stored.
 * @param roll Pointer to a float where the roll will be stored.
 * @return true if the orientation is retrieved successfully.
 * @return false if any error occurs during communication or data retrieval.
 */
bool getOrientationUSB(serial::Serial *s, float *yaw, float *pitch, float *roll)
{
    if (!s || !s->isOpen())
    {
        std::cerr << "❌ Serial port not open!" << std::endl;
        return FAILURE;
    }

    // Set IMU to binary output mode for orientation angles
    std::string setModeCmd = "#oscb";
    s->flushInput();
    s->write(setModeCmd);
    // Optional: wait a short delay for the IMU to process the command
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Request one output frame with orientation data
    std::string requestFrameCmd = "#f";
    s->flushInput();
    s->write(requestFrameCmd);

    // Expecting 12 bytes: 4 bytes each for yaw, pitch, and roll.
    uint8_t buffer[36] = {0};
    std::size_t bytesRead = s->read(buffer, 36);
    if (bytesRead != 36)
    {
        std::cerr << "❌ Failed to read orientation data from IMU. Expected 12 bytes, got "
                  << bytesRead << std::endl;
        return FAILURE;
    }

    // Convert the 12 bytes into 3 floats (assuming little-endian byte order)
    std::memcpy(yaw, buffer + 24, 4);
    std::memcpy(pitch, buffer + 28, 4);
    std::memcpy(roll, buffer + 32, 4);

    *yaw += 180;
    *pitch += 180;
    *roll += 180;

    return SUCCESS;
}

#endif //__INERTIAL_CONTROL_APIS__