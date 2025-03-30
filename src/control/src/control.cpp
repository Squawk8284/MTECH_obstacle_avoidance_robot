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

// User Defined Library
#include <ros_callbacks.hpp>

// Global Objects
serial::Serial *robotPort = nullptr;
serial::Serial *imuPort = nullptr;

int main(int argc, char **argv)
{

    try
    {
        robotPort = createSerial("/dev/ttyRobot", 57600);
        ros::init(argc, argv, "cmd_vel_listener");
        ros::NodeHandle nh;

        ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

        ros::spin();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        clearSerial(robotPort);
    }

    return 0;
}
