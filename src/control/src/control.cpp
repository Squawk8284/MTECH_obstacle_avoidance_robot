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

#include <user_defined_functions.hpp>

int main(int argc, char **argv)
{
    serial::Serial *robotPort = nullptr;
    try
    {
        robotPort = createSerial("/dev/ttyRobot", 57600);
        code();
        // init(robotPort);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        clearSerial(robotPort);
    }

    return 0;
}

void code()
{
    PrintLn("Hello World");
}