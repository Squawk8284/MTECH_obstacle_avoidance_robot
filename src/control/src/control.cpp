#include <iostream>
#include <nex_robot.hpp>
#include <motor_control_api.hpp>

int main(int argc, char **argv)
{
    serial::Serial *robotPort = createSerial("/dev/ttyRobot", 57600);
    if (!robotPort)
    {
        std::cerr << "One or more serial ports failed to open. Exiting." << std::endl;
        return -1;
    }

    std::vector<uint8_t> data;
    if (setRobotMode(robotPort, SAFETY_ON))
    {
        std::cout << "Robot Mode Response: ";
        for (auto byte : data)
            std::cout << std::hex << static_cast<int>(byte) << " ";
        std::cout << std::endl;
    }
    else
    {
        std::cerr << "Failed to set Robot Mode!" << std::endl;
    }

    clearSerial(robotPort);
    return 0;
}