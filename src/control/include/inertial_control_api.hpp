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

constexpr double g_to_m_s_2 = 9.81;
// ---------------------------------------------------------------------------
// Internal IMU Sensor Commands
// ---------------------------------------------------------------------------

/**
 * @brief Get 3 Axis Accelerometer data (m^s2)
 *
 * @param s Pointer to serial object
 * @param xAccel Pointer to X-axis Accelerometer data
 * @param yAccel Pointer to y-axis Accelerometer data
 * @param zAccel Pointer to z-axis Accelerometer data
 * @return true if Success
 * @return false if Failed
 */
bool get3AxisAccelorometer(serial::Serial *s, float *xAccel, float *yAccel, float *zAccel)
{
    uint8_t buffer[6];
    if (!executeCommand(s, CMD(get3AxisAccelorometer, 0x05, 0x05), nullptr, 0, ReturnPayload(6), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t rawXAccel = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);
    int16_t rawYAccel = (static_cast<int16_t>(buffer[3] << 8)) | (buffer[2]);
    int16_t rawZAccel = (static_cast<int16_t>(buffer[5] << 8)) | (buffer[4]);

    *xAccel = rawXAccel * (0.004 / 16) * g_to_m_s_2;
    *yAccel = rawYAccel * (0.004 / 16) * g_to_m_s_2;
    *zAccel = rawZAccel * (0.004 / 16) * g_to_m_s_2;
    return SUCCESS;
}

/**
 * @brief Get 3 Axis Gyroscope data (deg/sec)
 *
 * @param s Pointer to serial object
 * @param xGyro Pointer to X-axis Gyroscope data
 * @param yGyro Pointer to y-axis Gyroscope data
 * @param zGyro Pointer to z-axis Gyroscope data
 * @return true if Success
 * @return false if Failed
 */
bool get3AxisGyroscope(serial::Serial *s, float *xGyro, float *yGyro, float *zGyro)
{
    uint8_t buffer[6];
    if (!executeCommand(s, CMD(get3AxisGyroscope, 0x05, 0x06), nullptr, 0, ReturnPayload(6), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t rawXGyro = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);
    int16_t rawYGyro = (static_cast<int16_t>(buffer[3] << 8)) | (buffer[2]);
    int16_t rawZGyro = (static_cast<int16_t>(buffer[5] << 8)) | (buffer[4]);

    *xGyro = rawXGyro * (0.01 / 57.0);
    *yGyro = rawYGyro * (0.01 / 57.0);
    *zGyro = rawZGyro * (0.01 / 57.0);
    return SUCCESS;
}

/**
 * @brief Get 3 Axis Gyroscope data (Gauss)
 *
 * @param s Pointer to serial object
 * @param xMag Pointer to X-axis Magnetometer data
 * @param yMag Pointer to y-axis Magnetometer data
 * @param zMag Pointer to z-axis Magnetometer data
 * @return true if Success
 * @return false if Failed
 */
bool get3AxisMagnetometer(serial::Serial *s, float *xMag, float *yMag, float *zMag)
{
    uint8_t buffer[6];
    if (!executeCommand(s, CMD(get3AxisMagnetometer, 0x05, 0x07), nullptr, 0, ReturnPayload(6), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t rawXMag = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);
    int16_t rawYMag = (static_cast<int16_t>(buffer[3] << 8)) | (buffer[2]);
    int16_t rawZMag = (static_cast<int16_t>(buffer[5] << 8)) | (buffer[4]);

    *xMag = rawXMag / 1100.0;
    *yMag = rawYMag / 1100.0;
    *zMag = rawZMag / 980.0;
    return SUCCESS;
}

/**
 * @brief Get X Axis Accelerometer Data (m/s^2)
 *
 * @param s Pointer to serial Object
 * @param xAccel Pointer to X-axis Accelerometer data
 * @return true if Success
 * @return false if Failed
 */
bool getXAxisAccelerometer(serial::Serial *s, float *xAccel)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(getXAxisAccelerometer, 0x10, 0x01), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_data = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);

    *xAccel = raw_data * (0.004 / 16) * g_to_m_s_2;
    return SUCCESS;
}

/**
 * @brief Get Y Axis Accelerometer Data (m/s^2)
 *
 * @param s Pointer to serial Object
 * @param yAccel Pointer to Y-axis Accelerometer data
 * @return true if Success
 * @return false if Failed
 */
bool getYAxisAccelerometer(serial::Serial *s, float *yAccel)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(getYAxisAccelerometer, 0x10, 0x02), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_data = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);

    *yAccel = raw_data * (0.004 / 16) * g_to_m_s_2;
    return SUCCESS;
}

/**
 * @brief Get Z Axis Accelerometer Data (m/s^2)
 *
 * @param s Pointer to serial Object
 * @param zAccel Pointer to Z-axis Accelerometer data
 * @return true if Success
 * @return false if Failed
 */
bool getZAxisAccelerometer(serial::Serial *s, float *zAccel)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(getZAxisAccelerometer, 0x10, 0x03), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_data = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);

    *zAccel = raw_data * (0.004 / 16) * g_to_m_s_2;
    return SUCCESS;
}

/**
 * @brief Get X Axis Gyroscope Data (deg/sec)
 *
 * @param s Pointer to serial object
 * @param xGyro Pointer to x-axis Gyroscope data
 * @return true if Success
 * @return false if Failed
 */
bool getXAxisGyroscope(serial::Serial *s, float *xGyro)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(getXAxisGyroscope, 0x11, 0x01), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_data = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);
    *xGyro = raw_data * (0.01 / 57.0);
    return SUCCESS;
}

/**
 * @brief Get Y Axis Gyroscope Data (deg/sec)
 *
 * @param s Pointer to serial object
 * @param yGyro Pointer to y-axis Gyroscope data
 * @return true if Success
 * @return false if Failed
 */
bool getYAxisGyroscope(serial::Serial *s, float *yGyro)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(getYAxisGyroscope, 0x11, 0x02), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_data = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);
    *yGyro = raw_data * (0.01 / 57.0);
    return SUCCESS;
}

/**
 * @brief Get Z Axis Gyroscope Data (deg/sec)
 *
 * @param s Pointer to serial object
 * @param zGyro Pointer to z-axis Gyroscope data
 * @return true if Success
 * @return false if Failed
 */
bool getZAxisGyroscope(serial::Serial *s, float *zGyro)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(getZAxisGyroscope, 0x11, 0x03), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_data = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);
    *zGyro = raw_data * (0.01 / 57.0);
    return SUCCESS;
}

/**
 * @brief Get X Axis Magnetometer data (Gauss)
 * 
 * @param s Pointer to serial object
 * @param xMag Pointer to x-axis magnetometer data
 * @return true if success
 * @return false if failed
 */
bool getXAxisMagnetometer(serial::Serial *s, float *xMag)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(getXAxisMagnetometer, 0x12, 0x01), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_data = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);
    *xMag = raw_data / (1100.0);
    return SUCCESS;
}

/**
 * @brief Get Y Axis Magnetometer data (Gauss)
 * 
 * @param s Pointer to serial object
 * @param yMag Pointer to y-axis magnetometer data
 * @return true if success
 * @return false if failed
 */
bool getYAxisMagnetometer(serial::Serial *s, float *yMag)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(getYAxisMagnetometer, 0x12, 0x02), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_data = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);
    *yMag = raw_data / (1100.0);
    return SUCCESS;
}

/**
 * @brief Get Z Axis Magnetometer data (Gauss)
 * 
 * @param s Pointer to serial object
 * @param zMag Pointer to z-axis magnetometer data
 * @return true if success
 * @return false if failed
 */
bool getZAxisMagnetometer(serial::Serial *s, float *zMag)
{
    uint8_t buffer[2];
    if (!executeCommand(s, CMD(getZAxisMagnetometer, 0x12, 0x03), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    int16_t raw_data = (static_cast<int16_t>(buffer[1] << 8)) | (buffer[0]);
    *zMag = raw_data / (980.0);
    return SUCCESS;
}

/**
 * @brief Set the Buzzer
 *
 * @param s Pointer to Serial Object
 * @param buzzer BUZZER_ON=0x01 or BUZZER_OFF = 0x00
 * @return true
 * @return false
 */
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