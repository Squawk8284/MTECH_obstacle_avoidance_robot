/**
 * @file power_management_api.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief Power Management API for NEX robot using vector‐based commands.
 * @version 0.1
 * @date 2025-03-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __POWER_MANAGEMENT_APIS__
#define __POWER_MANAGEMENT_APIS__

#include <nex_robot.hpp>

// ---------------------------------------------------------------------------
// Power Control Commands
// ---------------------------------------------------------------------------

bool ReadBatteryVoltage(serial::Serial *s, float *voltage)
{
    uint8_t buffer = 0;
    if (!executeCommand(s, CMD(ReadBatteryVoltage, 0x20, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *voltage = (buffer * 0.14235) + 0.35;
    return SUCCESS;
}

bool ReadBatteryCurrent(serial::Serial *s, float *current)
{
    uint8_t buffer = 0;
    if (!executeCommand(s, CMD(ReadBatteryCurrent, 0x21, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *current = (2.5 - (buffer * 0.0129)) / 0.185;
    return SUCCESS;
}

bool ReadBatteryTemperature(serial::Serial *s, float *temperature)
{
    uint8_t buffer = 0;
    if (!executeCommand(s, CMD(ReadBatteryTemperature, 0x22, 0x00), nullptr, 0, ReturnPayload(1), &buffer, sizeof(buffer)))
        return FAILURE;
    *temperature = buffer * 1.29;
    return SUCCESS;
}

bool ReadBatteryVolCurTemp(serial::Serial *s, float *voltage, float *current, float *temperature)
{
    uint8_t buffer[3] = {0};
    if (!executeCommand(s, CMD(ReadBatteryVolCurTemp, 0x23, 0x00), nullptr, 0, ReturnPayload(3), buffer, sizeof(buffer)))
        return FAILURE;
    *voltage = (buffer[0] * 0.14235) + 0.35;
    *current = (2.5 - (buffer[1] * 0.0129)) / 0.185;
    *temperature = buffer[2] * 1.29;
    return SUCCESS;
}
#endif //__POWER_MANAGEMENT_APIS__