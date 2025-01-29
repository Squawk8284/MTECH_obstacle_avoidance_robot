#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdio.h>
#include <stdint.h>

#include <cmath>
#include <vector>
#include <cstring>

extern vector<pair<double, double>> bezierCurve;
extern RobotState state;
extern double P[5][5]; // Covariance matrix
extern double Q_k[5][5];
extern double R_k[2][2];

extern void *hSerial;
extern void *hIMU;
extern lib0xRobotCpp robot;

/************************************************************
 *   STRUCTURE DEFINATIONS
 *************************************************************/
typedef struct
{
    double x;
    double y;
    double theta;
    double v;
    double omega;

} RobotState;

typedef struct
{
    double omega_r;
    double omega_l;
} ControlInput;

typedef struct
{
    double v_imu;
    double omega_imu;

} SensorMeasurement;

/************************************************************
 *   FUNCTION DECLEARATIONS and Definations
 *************************************************************/

// Computing Bezier Curve
vector<pair<double, double>> computeBezierCurve(const vector<pair<double, double>> &controlPoints, int numPoints)
{
    vector<pair<double, double>> curve;

    int n = controlPoints.size() - 1;

    auto binomialCoeff = [](int n, int k)
    {
        double res = 1;
        for (int i = 0; i < k; i++)
        {
            res *= (n - i) / (i + 1.0);
        }
        return res;
    };

    for (int i = 0; i <= numPoints; i++)
    {
        double t = i / (double)numPoints;
        double x = 0, y = 0;
        for (int j = 0; j <= n; j++)
        {
            double coeff = binomialCoeff(n, j) * pow(t, j) * pow(1 - t, n - j);
            x += coeff * controlPoints[j].first;
            y += coeff * controlPoints[j].second;
        }

        curve.emplace_back(x, y);
    }

    return curve;
}

// Intialise EKF
void ekf_init()
{
    state = {bezierCurve[0].first, bezierCurve[0].second, 0.0, 0.0, 0.0};
    memcpy(P, Q_k, sizeof(P));
}

// Compute F_k Jacobian Matrix
void compute_F_k(double dt)
{
    // Jacobian matrix for the motion model
    double F_k[5][5] = {
        {1, 0, -dt * state.v * sin(state.theta + state.omega * dt), dt * cos(state.theta + state.omega * dt), 0},
        {0, 1, dt * state.v * cos(state.theta + state.omega * dt), dt * sin(state.theta + state.omega * dt), 0},
        {0, 0, 1, 0, dt},
        {0, 0, 0, 1, 0},  // Assuming no direct dependency of omega on state
        {0, 0, 0, 0, 1}   // Assuming constant angular velocity omega
    };

    // Propagate the covariance P
    double P_k[5][5];
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            P_k[i][j] = 0;
            for (int k = 0; k < 5; ++k) {
                P_k[i][j] += F_k[i][k] * P[k][j];
            }
        }
    }

    // P_k * F_k^T
    double P_new[5][5];
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            P_new[i][j] = 0;
            for (int k = 0; k < 5; ++k) {
                P_new[i][j] += P_k[i][k] * F_k[j][k];
            }
        }
    }

    // Update P with the new covariance
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            P[i][j] = P_new[i][j] + Q_k[i][j];  // Add process noise
        }
    }
}


// EKF Update Step
void ekf_update(SensorMeasurement z)
{
    double H_k[2][5] = {
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 1}};

    // Kalman Gain: K = P_k * H_k^T * (H_k * P_k * H_k^T + R_k)^-1
    double HT[5][2];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 2; j++) {
            HT[i][j] = H_k[j][i];
        }
    }

    // H_k * P_k
    double HP[2][5];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 5; j++) {
            HP[i][j] = 0;
            for (int k = 0; k < 5; k++) {
                HP[i][j] += H_k[i][k] * P[k][j];
            }
        }
    }

    // H_k * P_k * H_k^T
    double HPHt[2][2];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            HPHt[i][j] = 0;
            for (int k = 0; k < 5; k++) {
                HPHt[i][j] += HP[i][k] * HT[k][j];
            }
        }
    }

    // (H_k * P_k * H_k^T + R_k) inverse
    double S[2][2];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            S[i][j] = HPHt[i][j] + R_k[i][j];
        }
    }

    // Assuming S is invertible and using a basic 2x2 matrix inverse formula
    double detS = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    double S_inv[2][2] = {
        {S[1][1] / detS, -S[0][1] / detS},
        {-S[1][0] / detS, S[0][0] / detS}};

    // Kalman Gain K = P_k * H_k^T * S_inv
    double K[5][2];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 2; j++) {
            K[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                K[i][j] += P[i][k] * S_inv[k][j];
            }
        }
    }

    // Update state estimate
    state.v += K[3][0] * (z.v_imu - state.v);
    state.omega += K[4][1] * (z.omega_imu - state.omega);
}

// EKF Prediction Step
void ekf_predict(ControlInput u, double dt)
{
    compute_F_k(dt);
    state.x += state.v * cos(state.theta) * dt;
    state.y += state.v * sin(state.theta) * dt;
    state.theta += state.omega * dt;
}

// Move robot towards next point on Bézier curve
void moveRobot(double targetX, double targetY)
{
    double errorX = targetX - state.x;
    double errorY = targetY - state.y;
    double targetTheta = atan2(errorY, errorX);
    double angularError = targetTheta - state.theta;

    double linearVelocity = 0.1;
    double angularVelocity = angularError * 0.5;

    robot.setVelocity_meterspersec(hSerial, linearVelocity, linearVelocity);
    robot.setRobotAngularVelocityRadianpersec(hSerial, angularVelocity);
}
#endif