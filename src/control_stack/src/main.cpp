#include "0xRobotcpplib.h"
#include <cmath>
#include <vector>
#include <cstring>
#include <iostream>

using namespace std;

lib0xRobotCpp robot;

int main()
{
    int16 gyroXYZ_zero[3];
    float magValue[3];
    float accelValue[3];

    robot.comm_handle = robot.connect_comm("/dev/ttyRobot");
    
    robot.stop(robot.comm_handle);
    robot.resetMotorEncoderCount(robot.comm_handle);
    robot.setAcceleration(robot.comm_handle, 4);
    robot.setLinearVelocity_meterspersec(robot.comm_handle, 0.25);
    robot.setSafetyTimeout(robot.comm_handle, 0);
    robot.setSafety(robot.comm_handle, 0);

    robot.backward(robot.comm_handle);
    robot.DelaymSec(robot.comm_handle,2000);

    robot.imuInit(robot.comm_handle,gyroXYZ_zero);
    robot.getTiltHeading(robot.comm_handle,magValue,accelValue);

    cout<<"Mag Value: "<<magValue[0]<<", "<<magValue[1]<<", "<<magValue[2]<<endl;
    cout<<"Accel Value: "<<accelValue[0]<<", "<<accelValue[1]<<", "<<accelValue[2]<<endl;

    robot.stop(robot.comm_handle);
    robot.backward(robot.comm_handle);
    robot.DelaymSec(robot.comm_handle,1000);
    robot.stop(robot.comm_handle);
    printf("Hello World");
    return 0;
}