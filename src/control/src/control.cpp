/**
 * @file control.cpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief Main file for the control stack (low level control for the robot)
 * @version 0.1
 * @date 2025-03-26
 *
 * @copyright Copyright (c) 2025
 *
 */

// #define DEBUG   //Uncomment to enable Debugging

#include <iostream>
#include <nex_robot.hpp>
#include <motor_control_api.hpp>
#include <power_management_api.hpp>
#include <inertial_control_api.hpp>

int main(int argc, char **argv)
{
    serial::Serial *robotPort = createSerial("/dev/ttyRobot", 57600);

    float vol, cur, temp;
    if (!setSafetyTimeout(robotPort, 10.1))
        ;
    if (!ReadBatteryVolCurTemp(robotPort, &vol, &cur, &temp))
        ;
    PrintLn("Return Value = ", vol, ",", cur, ",", temp);

    clearSerial(robotPort);
    return 0;
}