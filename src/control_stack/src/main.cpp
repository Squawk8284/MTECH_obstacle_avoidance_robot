#include <stdio.h>
#include <stdint.h>
#include <cmath>
#include <vector>
#include <cstring>

#include <ros/ros.h>
#include "0xRobotcpplib.h"

#include "control.h"

using namespace std;

/* VARIABLE DECLARATIONS*/
lib0xRobotCpp robot;

void *hSerial = robot.connect_comm("/dev/ttyRobot");
void *hIMU = robot.connect_comm("/dev/ttyIMU");

RobotState state;
double P[5][5]; // Covariance matrix

double Q_k[5][5] = { // Noise Matrix (from Paper)
    {0.1, 0, 0, 0, 0},
    {0, 0.1, 0, 0, 0},
    {0, 0, 0.1, 0, 0},
    {0, 0, 0, 0.1, 0},
    {0, 0, 0, 0, 0.1}};

// Measurement noise covariance matrix R (from paper)
double R_k[2][2] = {
    {0.018, 0},
    {0, 0.018}};

vector<pair<double, double>> bezierCurve;

int main(int argc, char **argv)
{
    vector<pair<double, double>> controlPoints = {{0, 0}, {1, 2}, {3, 3}, {4, 2}, {5, 0}};
    bezierCurve = computeBezierCurve(controlPoints, 100);
    ekf_init();

    int pointIndex = 0;
    while (pointIndex < bezierCurve.size())
    {
        ControlInput u;
        robot.getRightMotorVelocity(hSerial, (signed short)u.omega_r);
        robot.getLeftMotorVelocity(hSerial, (signed short)u.omega_l);

        SensorMeasurement z;
        robot.getAccelerometerX(hIMU, (int16 *)&z.v_imu);
        robot.getGyroZ(hIMU, (int16 *)&z.omega_imu);

        ekf_update(z);
        ekf_predict(u, 0.1);

        double targetX = bezierCurve[pointIndex].first;
        double targetY = bezierCurve[pointIndex].second;
        moveRobot(targetX, targetY);

        if (hypot(state.x - targetX, state.y - targetY) < 0.1)
        {
            pointIndex++;
        }
    }
    robot.stop(hSerial);
    return 0;
}
