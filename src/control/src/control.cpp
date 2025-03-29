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
#include <inertial_control_api.hpp>

int main(int argc, char **argv)
{
    serial::Serial *robotPort = createSerial("/dev/ttyRobot", 57600);
    serial::Serial *imuPort = createSerial("/dev/ttyIMU", 57600);
    if (!robotPort || !imuPort)
    {
        std::cerr << "One or more serial ports failed to open. Exiting." << std::endl;
        return -1;
    }

    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
    std::vector<uint8_t> data;

    float velocity = 0.4;
    // if (!setSafetyTimeout(robotPort, 1))
    //     ;
    // if (!getSafetyTimeout(robotPort, &data))
    //     ;
    std::cout << "Safety Timeout = ";
    for (auto byte : data)
        std::cout << static_cast<float>(byte) << " ";
    std::cout << std::endl;
    // if(!setLeftMotorVelocity_mps(robotPort, &velocity));
    // if(!setRightMotorVelocity_mps(robotPort, &velocity));
    // if(!setRobotDirection(robotPort, 1));

    if (getOrientationUSB(imuPort, &yaw, &pitch, &roll))
    {
        std::cout << "Orientation:" << std::endl;
        std::cout << "Yaw  : " << yaw << " degrees" << std::endl;
        std::cout << "Pitch: " << pitch << " degrees" << std::endl;
        std::cout << "Roll : " << roll << " degrees" << std::endl;
    }
    else
    {
        std::cerr << "Failed to retrieve orientation data." << std::endl;
    }

    clearSerial(robotPort);
    clearSerial(imuPort);
    return 0;
}