/**
 * @file servo_pods_api.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief Servo Pods API for nex robot 0xDelta
 * @version 0.1
 * @date 2025-05-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __SERVO_PODS_API_H__
#define __SERVO_PODS_API_H__

#include <utils.hpp>

// ---------------------------------------------------------------------------
// Servo pods Commands
// ---------------------------------------------------------------------------

/**
 * @brief Set the Servo Pod Pan Angle (0-180) deg
 *
 * @param s Pointer to Serial Object
 * @param PanAngle Pan Angle (0-180) deg
 * @return true if successful
 * @return false if failed
 */
bool setServoPodPanAngle(serial::Serial *s, uint8_t PanAngle)
{
    if (!executeCommand(s, CMD(setServoPodPanAngle, 0x06), &PanAngle, sizeof(PanAngle), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Set the Servo Pod Tilt Angle
 *
 * @param s Pointer to Serial Object
 * @param TiltAngle Tilt Angle (0-180) deg
 * @return true if successful
 * @return false if failed
 */
bool setServoPodTiltAngle(serial::Serial *s, uint8_t TiltAngle)
{
    if (!executeCommand(s, CMD(setServoPodTiltAngle, 0x07), &TiltAngle, sizeof(TiltAngle), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Set the Servo Pod Aux Angle
 *
 * @param s Pointer to Serial Object
 * @param AuxAngle Aux Angle (0-180) deg
 * @return true if successful
 * @return false if failed
 */
bool setServoPodAuxAngle(serial::Serial *s, uint8_t AuxAngle)
{
    if (!executeCommand(s, CMD(setServoPodAuxAngle, 0x08), &AuxAngle, sizeof(AuxAngle), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Get the Servo Pod Pan Angle
 *
 * @param s Pointer to Serial Object
 * @param PanAngle Pointer Pan Angle Object
 * @return true if successful
 * @return false if failed
 */
bool getServoPodPanAngle(serial::Serial *s, uint8_t *PanAngle)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(getServoPodPanAngle, 0xAA, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    *PanAngle = buffer;
    return SUCCESS;
}

/**
 * @brief Get the Servo Pod Tilt Angle
 *
 * @param s Pointer to Serial Object
 * @param TiltAngle Pointer Tilt Angle Object
 * @return true if successful
 * @return false if failed
 */
bool getServoPodTiltAngle(serial::Serial *s, uint8_t *TiltAngle)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(getServoPodTiltAngle, 0xAB, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    *TiltAngle = buffer;
    return SUCCESS;
}

/**
 * @brief Get the Servo Pod Aux Angle
 *
 * @param s Pointer to Serial Object
 * @param AuxAngle Pointer Aux Angle Object
 * @return true if successful
 * @return false if failed
 */
bool getServoPodAuxAngle(serial::Serial *s, uint8_t *AuxAngle)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(getServoPodAuxAngle, 0xAC, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    *AuxAngle = buffer;
    return SUCCESS;
}

/**
 * @brief Get the Servo Pod Ultrasonic Data
 * 
 * @param s Pointer to Serial Object
 * @param UltrasonicData 16 bit Ultrasonic Data
 * @return true if successful
 * @return false if failed 
 */
bool getServoPodUltrasonicData(serial::Serial *s, uint16_t *UltrasonicData)
{
    uint8_t buffer[2];

    if (!executeCommand(s, CMD(getServoPodUltrasonicData, 0x05, 0x0A), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    *UltrasonicData = (static_cast<int16_t>(buffer[0] << 8)) | buffer[1];

    return SUCCESS;
}
#endif //__SERVO_PODS_API_H__