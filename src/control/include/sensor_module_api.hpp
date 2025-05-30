/**
 * @file sensor_module_api.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-05-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __SENSOR_MODULE_APIS__
#define __SENSOR_MODULE_APIS__

#include <utils.hpp>

// IR Proximity
#define IR_PROXIMITY_SENSOR_ON (0x01)
#define IR_PROXIMITY_SENSOR_OFF (0x00)

// White Line
#define WHITE_LINE_SENSOR_ON (0x01)
#define WHITE_LINE_SENSOR_OFF (0x00)

// Ultrasonic Sensor Ranging
#define ULTRASONIC_RANGING_SENSOR_ON (0x01)
#define ULTRASONIC_RANGING_SENSOR_OFF (0x00)

// Servo pod Ultrasonic Ranging
#define SERVO_POD_ULTRASONIC_RANGING_SENSOR_ON (0x01)
#define SERVO_POD_ULTRASONIC_RANGING_SENSOR_OFF (0x00)

// IR Distance
#define IR_DISTANCE_SENSOR_ON (0x01)
#define IR_DISTANCE_SENSOR_OFF (0x00)

// ---------------------------------------------------------------------------
// Sensor Module Commands
// ---------------------------------------------------------------------------

/**
 * @brief Get 8 Bytes of Ultrasonic Data
 *
 * @param s Pointer to Serial Object
 * @param RawUltrasonicData  Pointer to raw 8 byte vector or buffer
 * @return true if successful
 * @return false if failed
 */
bool get8bytesofUltrasonicData(serial::Serial *s, float *RawUltrasonicData)
{
    uint8_t buffer[8];
    if (!executeCommand(s, CMD(get8bytesofUltrasonicData, 0x05, 0x01), nullptr, 0, ReturnPayload(8), buffer, sizeof(buffer)))
        return FAILURE;

    std::memcpy(RawUltrasonicData, buffer, sizeof(buffer));

    return SUCCESS;
}

/**
 * @brief Get the Single Ultrasonic Data
 *
 * @param s Pointer to Serial Object
 * @param RawUltrasonicData Variable to store the ultrasonic raw data
 * @param SensorNumber Sensor Number for which the data is required (0-7 only)
 * @return true if successful
 * @return false if failed
 *
 * 1. LV-MAXSonar-EZ4 Ultrasonic data * 5 = Distance in cm
 * 2. HRXL-MaxSonar-WRT (MB7380) Ultrasonic data * 20 = Distance in mm
 * 3. HRXL-MaxSonar-WRLS (MB7363) Ultrasonic data * 40 = Distance in mm
 */
bool getSingleUltrasonicData(serial::Serial *s, float *RawUltrasonicData, int8_t SensorNumber)
{
    if (SensorNumber > 7 || SensorNumber < 0)
    {
        _DEBUG(PrintLn("Sensor Number is not Connected"))
        return FAILURE;
    }

    uint8_t buffer;
    if (!executeCommand(s, CMD(getSingleUltrasonicData, 0x01), &SensorNumber, sizeof(SensorNumber), ReturnPayload(1), &buffer, sizeof(buffer)))
    {
        PrintLn("Unable to fetch sensor data for ", SensorNumber);
        return FAILURE;
    }

    *RawUltrasonicData = static_cast<float>(buffer);
    return SUCCESS;
}

/**
 * @brief Get 8 Bytes of Line Sensor Data
 *
 * @param s Pointer to Serial Object
 * @param RawLineSensorData  Pointer to raw 8 byte vector or buffer
 * @return true if successful
 * @return false if failed
 */
bool get8bytesofLineSensorData(serial::Serial *s, float *RawLineSensorData)
{
    uint8_t buffer[8];
    if (!executeCommand(s, CMD(get8bytesofLineSensorData, 0x05, 0x04), nullptr, 0, ReturnPayload(8), buffer, sizeof(buffer)))
        return FAILURE;

    std::memcpy(RawLineSensorData, buffer, sizeof(buffer));

    return SUCCESS;
}

/**
 * @brief Get the Single Line Sensor Data
 *
 * @param s Pointer to Serial Object
 * @param RawLineSensorData Variable to Store the Line Sensor raw Data
 * @param SensorNumber Sensor Number for which the data is required (0-7 only)
 * @return true if successful
 * @return false if failed
 */
bool getSingleLineSensorData(serial::Serial *s, float *RawLineSensorData, int8_t SensorNumber)
{
    if (SensorNumber > 7 || SensorNumber < 0)
    {
        _DEBUG(PrintLn("Sensor Number is not Connected"))
        return FAILURE;
    }

    uint8_t buffer;
    if (!executeCommand(s, CMD(getSingleLineSensorData, 0x04), &SensorNumber, sizeof(SensorNumber), ReturnPayload(1), &buffer, sizeof(buffer)))
    {
        PrintLn("Unable to fetch sensor data for ", SensorNumber);
        return FAILURE;
    }

    *RawLineSensorData = static_cast<float>(buffer);
    return SUCCESS;
}

/**
 * @brief Get 8 Bytes of Proximity Sensor Data
 *
 * @param s Pointer to Serial Object
 * @param RawProximitySensorData  Pointer to raw 8 byte vector or buffer
 * @return true if successful
 * @return false if failed
 */
bool get8bytesofProximtySensorData(serial::Serial *s, float *RawProximitySensorData)
{
    uint8_t buffer[8];
    if (!executeCommand(s, CMD(get8bytesofProximtySensorData, 0x05, 0x02), nullptr, 0, ReturnPayload(8), buffer, sizeof(buffer)))
        return FAILURE;

    std::memcpy(RawProximitySensorData, buffer, sizeof(buffer));

    return SUCCESS;
}

/**
 * @brief Get the Single Proximity Sensor Data
 *
 * @param s Pointer to Serial Object
 * @param RawProximitySensorData Variable to Store the Proximity Sensor raw Data
 * @param SensorNumber Sensor Number for which the data is required (0-7 only)
 * @return true if successful
 * @return false if failed
 */
bool getSingleProximityData(serial::Serial *s, float *RawProximitySensorData, int8_t SensorNumber)
{
    if (SensorNumber > 7 || SensorNumber < 0)
    {
        _DEBUG(PrintLn("Sensor Number is not Connected"))
        return FAILURE;
    }

    uint8_t buffer;
    if (!executeCommand(s, CMD(getSingleProximityData, 0x02), &SensorNumber, sizeof(SensorNumber), ReturnPayload(1), &buffer, sizeof(buffer)))
    {
        PrintLn("Unable to fetch sensor data for ", SensorNumber);
        return FAILURE;
    }

    *RawProximitySensorData = static_cast<float>(buffer);
    return SUCCESS;
}

/**
 * @brief Get 8 Bytes of Distance Sensor Data
 *
 * @param s Pointer to Serial Object
 * @param RawDistanceSensorData  Pointer to raw 8 byte vector or buffer
 * @return true if successful
 * @return false if failed
 */
bool get8bytesofDistanceSensorData(serial::Serial *s, float *RawDistanceSensorData)
{
    uint8_t buffer[8];
    if (!executeCommand(s, CMD(get8bytesofDistanceSensorData, 0x05, 0x03), nullptr, 0, ReturnPayload(8), buffer, sizeof(buffer)))
        return FAILURE;

    std::memcpy(RawDistanceSensorData, buffer, sizeof(buffer));

    return SUCCESS;
}

/**
 * @brief Get the Single Distance Sensor Data
 *
 * @param s Pointer to Serial Object
 * @param RawDistanceSensorData Variable to Store the Proximity Sensor raw Data
 * @param SensorNumber Sensor Number for which the data is required (0-7 only)
 * @return true if successful
 * @return false if failed
 */
bool getSingleDistanceData(serial::Serial *s, float *RawDistanceSensorData, int8_t SensorNumber)
{
    if (SensorNumber > 7 || SensorNumber < 0)
    {
        _DEBUG(PrintLn("Sensor Number is not Connected"))
        return FAILURE;
    }

    uint8_t buffer;
    if (!executeCommand(s, CMD(getSingleDistanceData, 0x03), &SensorNumber, sizeof(SensorNumber), ReturnPayload(1), &buffer, sizeof(buffer)))
    {
        PrintLn("Unable to fetch sensor data for ", SensorNumber);
        return FAILURE;
    }

    *RawDistanceSensorData = static_cast<float>(buffer);
    return SUCCESS;
}

/**
 * @brief Turn on the IR Proximity Sensor if connected
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOnIRProximitySensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOnIRProximitySensor, 0x09, 0x01), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Turn off the IR Proximity Sensor if connected
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOFFIRProximitySensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOFFIRProximitySensor, 0x09, 0x02), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Turn on the white line sensor if connected
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOnWhiteLineSensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOnWhiteLineSensor, 0x09, 0x03), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Turn off the white line sensor if connected
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOFFWhiteLineSensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOFFWhiteLineSensor, 0x09, 0x04), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Turn on the ultrasonic Ranging Sensor
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOnUltrasonicRangingSensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOnUltrasonicRangingSensor, 0x09, 0x05), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Turn off the ultrasonic Ranging Sensor
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOFFUltrasonicRangingSensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOFFUltrasonicRangingSensor, 0x09, 0x06), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Turn on the Servo pod ultrasonic Ranging Sensor
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOnServoPodUltrasonicRangingSensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOnServoPodUltrasonicRangingSensor, 0x09, 0x07), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Turn off the Servo pod ultrasonic Ranging Sensor
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOFFServoPodUltrasonicRangingSensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOFFServoPodUltrasonicRangingSensor, 0x09, 0x08), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Turn on the IR Distance Ranging Sensor if connected
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOnIRDistanceRangingSensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOnIRProximitySensor, 0x09, 0x09), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Turn off the IR Distance Ranging Sensor if connected
 *
 * @param s Pointer to Serial Object
 * @return true if successful
 * @return false if failed
 */
bool TurnOFFIRDistanceRangingSensor(serial::Serial *s)
{
    if (!executeCommand(s, CMD(TurnOFFIRProximitySensor, 0x09, 0x0A), nullptr, 0, ReturnPayload(0), nullptr, 0))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Get IR Proximity Sensor ON/OFF Status
 *
 * @param s Pointer to Serial Object
 * @param IRProximitySensorStatus Pointer to Proximity Sensor Status Variable
 * @return true if successful
 * @return false if failed
 */
bool getIRProximitySensorStatus(serial::Serial *s, int8_t *IRProximitySensorStatus)
{
    int8_t buffer;
    if (!executeCommand(s, CMD(getIRProximitySensorStatus, 0x0A, 0x01), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *IRProximitySensorStatus = buffer;
    return SUCCESS;
}

/**
 * @brief Get White Line Sensor ON/OFF Status
 *
 * @param s Pointer to Serial Object
 * @param WhiteLineSensorStatus Pointer to White Line Sensor Status Variable
 * @return true if successful
 * @return false if failed
 */
bool getWhiteLineSensorStatus(serial::Serial *s, int8_t *WhiteLineSensorStatus)
{
    int8_t buffer;
    if (!executeCommand(s, CMD(getWhiteLineSensorStatus, 0x0A, 0x02), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *WhiteLineSensorStatus = buffer;
    return SUCCESS;
}

/**
 * @brief Get Ultrasonic Sensor ON/OFF Status
 *
 * @param s Pointer to Serial Object
 * @param UltrasonicSensorStatus Pointer to Ultrasonic Sensor Status Variable
 * @return true if successful
 * @return false if failed
 */
bool getUltrasonicSensorStatus(serial::Serial *s, int8_t *UltrasonicSensorStatus)
{
    int8_t buffer;
    if (!executeCommand(s, CMD(getUltrasonicSensorStatus, 0x0A, 0x03), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *UltrasonicSensorStatus = buffer;
    return SUCCESS;
}

/**
 * @brief Get Servo Pod Ultrasonic Sensor ON/OFF Status
 *
 * @param s Pointer to Serial Object
 * @param ServoPodUltrasonicSensorStatus Pointer to Servo Pod Ultrasonic Sensor Status Variable
 * @return true if successful
 * @return false if failed
 */
bool getServoPodUltrasonicSensorStatus(serial::Serial *s, int8_t *ServoPodUltrasonicSensorStatus)
{
    int8_t buffer;
    if (!executeCommand(s, CMD(getServoPodUltrasonicSensorStatus, 0x0A, 0x04), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *ServoPodUltrasonicSensorStatus = buffer;
    return SUCCESS;
}

/**
 * @brief Get IR Distance Sensor ON/OFF Status
 *
 * @param s Pointer to Serial Object
 * @param IRDistanceSensorStatus Pointer to IR Distance Sensor Status Variable
 * @return true if successful
 * @return false if failed
 */
bool getIRDistanceSensorStatus(serial::Serial *s, int8_t *IRDistanceSensorStatus)
{
    int8_t buffer;
    if (!executeCommand(s, CMD(getIRDistanceSensorStatus, 0x0A, 0x05), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *IRDistanceSensorStatus = buffer;
    return SUCCESS;
}
#endif //__SENSOR_MODULE_APIS__