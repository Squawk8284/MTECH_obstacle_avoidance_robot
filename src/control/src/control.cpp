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
    int16_t velocity =1;
    if(setSafetyTimeout(robotPort, 3))
    if(setLeftMotorVelocity_mps(robotPort, &velocity));
    if(setRightMotorVelocity_mps(robotPort, &velocity));
    if(setRobotDirection(robotPort, 1));

    clearSerial(robotPort);
    return 0;
}