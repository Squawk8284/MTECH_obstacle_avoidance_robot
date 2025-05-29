/**
 * @file miscellaneous_api.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief Miscellaneous Commands for Nex Robot 0xDelta
 * @version 0.1
 * @date 2025-05-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __MISCELLANEOUS_COMMANDS_API_H__
#define __MISCELLANEOUS_COMMANDS_API_H__

#include <utils.hpp>

// ---------------------------------------------------------------------------
// Miscellaneous Commands
// ---------------------------------------------------------------------------

/**
 * @brief Set the Robot ID
 *
 * @param s Pointer to the serial object.
 * @param RobotID Set ID of the Robot
 * @return true if successful.
 * @return false if failed.
 */
bool setRobotID(serial::Serial *s, uint8_t RobotID)
{

    if (!executeCommand(s, CMD(setRobotID, 0xF9), &RobotID, sizeof(RobotID), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Get the Robot I D object
 *
 * @param s Pointer to the serial object.
 * @param RobotID Get ID of the Robot
 * @return true if successful.
 * @return false if failed.
 */
bool getRobotID(serial::Serial *s, uint8_t *RobotID)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(getRobotID, 0xFA, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Get the Hardware Version ID
 *
 * @param s Pointer to Serial Object
 * @param HardwareVersionID Harware version ID
 * @return true if successful.
 * @return false if failed.
 */
bool getHardwareVersionID(serial::Serial *s, uint8_t *HardwareVersionID)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(getHardwareVersionID, 0xFB, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Get the Software Version ID
 *
 * @param s Pointer to Serial Object
 * @param SoftwareVersionID Software Version ID
 * @return true if successful.
 * @return false if failed.
 */
bool getSoftwareVersionID(serial::Serial *s, uint8_t *SoftwareVersionID)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(getSoftwareVersionID, 0xFC, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    return SUCCESS;
}

/**
 * @brief Get the 12 bit Potentiometer Data
 *
 * @param s Pointer to Serial Object
 * @param Potentiometer
 * @return true if successful.
 * @return false if failed.
 */
bool getPotentiometerData(serial::Serial *s, int16_t *PotentiometerData)
{
    uint8_t buffer[2];

    if (!executeCommand(s, CMD(getPotentiometerData, 0x05, 0x09), nullptr, 0, ReturnPayload(2), buffer, sizeof(buffer)))
        return FAILURE;

    *PotentiometerData = (static_cast<int16_t>(buffer[0] << 8)) | buffer[1];

    return SUCCESS;
}

/**
 * @brief Get 16 Bytes AD7998 Data
 *
 * @param s Pointer to Serial Object
 * @param PointerTo16ByteDataBuffer Pointer to 16 bytes buffer object for AD7998 Data.
 * Byte0 = CH0_LSB & Byte 1 = CH0_MSB and so on.
 * @return true if successful.
 * @return false if failed.
 */
bool get16BytesAD7998Data(serial::Serial *s, uint8_t *PointerTo16ByteDataBuffer)
{
    uint8_t buffer[16];
    if (!executeCommand(s, CMD(get16BytesAD7998Data, 0x05, 0x08), nullptr, 0, ReturnPayload(16), buffer, sizeof(buffer)))
        return FAILURE;

    memcpy(PointerTo16ByteDataBuffer, buffer, sizeof(buffer));

    return SUCCESS;
}

/**
 * @brief Set GPIO Panel LED
 *
 * @param s Pointer to Serial Object
 * @param LEDData if LED_DATA = 0x05; then all 1 bits will set the LED
 * @return true if successful.
 * @return false if failed.
 */
bool setGPIOPanelLED(serial::Serial *s, uint8_t LEDData)
{
    if (!executeCommand(s, CMD(setGPIOPanelLED, 0x60), &LEDData, sizeof(LEDData), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}

/**
 * @brief Get LED Panel Status
 *
 * @param s Pointer to Serial Object
 * @param LEDData Pointer to LED data variable
 * @return true if successful.
 * @return false if failed.
 */
bool getGPIOPanelLEDStatus(serial::Serial *s, uint8_t *LEDData)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(getGPIOPanelLEDStatus, 0x61, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    *LEDData = buffer;
    return SUCCESS;
}

/**
 * @brief Get LED Panel Switch Status
 * 
 * @param s Pointer to Serial Object
 * @param LEDPanelSwitchData Pointer to LED Switch Data Variable
 * @return true if successful.
 * @return false if failed. 
 */
bool getLEDPanelSwitchStatus(serial::Serial *s, uint8_t *LEDPanelSwitchData)
{
    uint8_t buffer;
    if (!executeCommand(s, CMD(getLEDPanelSwitchStatus, 0x62, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;

    *LEDPanelSwitchData = buffer;
    return SUCCESS;
}

/**
 * @brief Set the Buzzer
 *
 * @param s Pointer to Serial Object
 * @param buzzer BUZZER_ON=0x01 or BUZZER_OFF = 0x00
 * @return true if successful
 * @return false if failed
 */
bool setBuzzer(serial::Serial *s, uint8_t buzzer)
{
    if (!executeCommand(s, CMD(SetBuzzer, 0x30), &buzzer, sizeof(buzzer), ReturnPayload(0), nullptr, 0))
        return FAILURE;
    return SUCCESS;
}
#endif // __MISCELLANEOUS_COMMANDS_API_H__