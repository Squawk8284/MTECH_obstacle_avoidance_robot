#include <iostream>
#include <vector>
#include <cmath>
#include "0xRobotcpplib.h"   // Include the robot's library header

using namespace std;

//---------------------------------------------------------
// Data Structures and Global Variables
//---------------------------------------------------------

// A simple 2D point structure (units: mm)
struct Point {
    double x; // in mm
    double y; // in mm
};

// Global control points for the Bézier curve (example values in mm)
// Modify these as needed.
vector<Point> controlPoints = {
    {1000, 1000},   // Starting point (mm)
    {1500, 2500},   // Control point 1 (mm)
    {2500, 2500},   // Control point 2 (mm)
    {3000, 1000}    // End point (mm)
};

//---------------------------------------------------------
// Bézier Curve Computation Functions
//---------------------------------------------------------

// Helper function: Compute factorial of n (for small n)
unsigned long factorial(unsigned int n) {
    unsigned long result = 1;
    for (unsigned int i = 2; i <= n; i++)
        result *= i;
    return result;
}

// Helper function: Compute binomial coefficient ("n choose i")
unsigned long binomialCoeff(unsigned int n, unsigned int i) {
    return factorial(n) / (factorial(i) * factorial(n - i));
}

// Compute a point on a Bézier curve given parameter t (0 <= t <= 1)
// using the control points provided.
Point computeBezierPoint(double t, const vector<Point>& ctrlPts) {
    int n = ctrlPts.size() - 1; // Degree of the Bézier curve
    Point result = {0.0, 0.0};

    for (int i = 0; i <= n; i++) {
        double bernstein = binomialCoeff(n, i) * pow(1 - t, n - i) * pow(t, i);
        result.x += bernstein * ctrlPts[i].x;
        result.y += bernstein * ctrlPts[i].y;
    }
    return result;
}

//---------------------------------------------------------
// Robot Localization (Dead-Reckoning Using Encoders)
//---------------------------------------------------------

// Global variables to hold previous encoder counts.
int32 prevLeftEncoder = 0;
int32 prevRightEncoder = 0;

// Update the robot’s global position using encoder counts.
// All positions are in millimeters (mm) and angles in radians.
// Parameters:
//   robot            - the robot object
//   comm             - communication handle
//   x, y, theta      - current global position and heading (theta: in radians)
//   distancePerCount - conversion factor (mm per encoder count)
//   axleLength       - distance between wheels in mm
void updateRobotPosition(lib0xRobotCpp &robot, void* comm,
                           double &x, double &y, double &theta,
                           double distancePerCount, double axleLength) 
{
    int32 currentLeftEncoder = 0;
    int32 currentRightEncoder = 0;

    // Get current encoder counts
    robot.getLeftMotorCount(comm, &currentLeftEncoder);
    robot.getRightMotorCount(comm, &currentRightEncoder);

    // Compute change in encoder counts
    int32 dLeft = currentLeftEncoder - prevLeftEncoder;
    int32 dRight = currentRightEncoder - prevRightEncoder;

    // Update previous counts for next iteration
    prevLeftEncoder = currentLeftEncoder;
    prevRightEncoder = currentRightEncoder;

    // Convert counts to distances (mm)
    double dLeft_mm = dLeft * distancePerCount;
    double dRight_mm = dRight * distancePerCount;

    // Compute forward displacement (mm) and change in orientation (radians)
    double dCenter = (dLeft_mm + dRight_mm) / 2.0;
    double dTheta = (dRight_mm - dLeft_mm) / axleLength;

    // Update global position using the average heading during the motion
    double thetaMid = theta + dTheta / 2.0;
    x += dCenter * cos(thetaMid);
    y += dCenter * sin(thetaMid);
    theta += dTheta;

    // Normalize theta to the range (-pi, pi)
    while (theta > M_PI)  theta -= 2 * M_PI;
    while (theta < -M_PI) theta += 2 * M_PI;
}

//---------------------------------------------------------
// Control Law (Pure Pursuit Style)
//---------------------------------------------------------

// Compute the linear velocity (v, in m/s) and angular velocity (w, in rad/s)
// based on the error between the target point and the robot’s current position.
// Global positions are in mm so conversion to meters is done when needed.
void computeControlSignals(double targetX, double targetY,
                           double currentX, double currentY, double currentTheta,
                           double &v, double &w) 
{
    // Compute the error vector (in mm)
    double errorX = targetX - currentX;
    double errorY = targetY - currentY;
    
    // Compute the Euclidean distance error (mm)
    double distanceError = sqrt(errorX * errorX + errorY * errorY);
    
    // Compute the desired heading toward the target (radians)
    double desiredTheta = atan2(errorY, errorX);
    
    // Compute the heading error (radians)
    double thetaError = desiredTheta - currentTheta;
    while (thetaError > M_PI)  thetaError -= 2 * M_PI;
    while (thetaError < -M_PI) thetaError += 2 * M_PI;
    
    // --- Controller Gains (tune these parameters) ---
    // For linear velocity: proportional to the distance error (converted from mm to m)
    const double Kp_v = 0.5;      // [m/s per m error]
    // For angular velocity: proportional to the heading error
    const double Kp_w = 2.0;      // [rad/s per rad error]

    // Compute control signals:
    // Convert distance error from mm to m (divide by 1000)
    v = Kp_v * (distanceError / 1000.0);
    // Limit the linear velocity to a maximum value (in m/s)
    const double max_v = 0.3; 
    if(v > max_v) v = max_v;

    w = Kp_w * thetaError;
    // Limit the angular velocity (in rad/s)
    const double max_w = 1.0;
    if (w > max_w)  w = max_w;
    if (w < -max_w) w = -max_w;
}

//---------------------------------------------------------
// Convert Control Signals to Wheel Velocities and Command Robot
//---------------------------------------------------------

// Converts a desired linear velocity (v, m/s) and angular velocity (w, rad/s)
// into individual left/right wheel speeds and sends the commands to the robot.
// The conversion uses the differential drive kinematics.
// Parameters:
//   robot       - the robot object
//   comm        - communication handle
//   v           - linear velocity (m/s)
//   w           - angular velocity (rad/s)
//   wheelRadius - radius of the wheels in meters
//   axleLength  - distance between wheels in mm (converted to m inside)
void setWheelVelocities(lib0xRobotCpp &robot, void* comm,
                        double v, double w,
                        double wheelRadius, double axleLength_mm) 
{
    // Convert axle length from mm to m
    double axleLength = axleLength_mm / 1000.0;
    
    // Compute left and right wheel velocities (in m/s) using differential drive kinematics
    double v_left = v - (w * axleLength / 2.0);
    double v_right = v + (w * axleLength / 2.0);

    // Send the velocity commands to the robot
    robot.setLeftMotorVelocity_meterspersec(comm, v_left);
    robot.setRightMotorVelocity_meterspersec(comm, v_right);
}

//---------------------------------------------------------
// Main Control Loop: Follow the Global Bézier Curve
//---------------------------------------------------------

// Makes the robot follow the Bézier curve in the global coordinate system.
// All measurements (control points, distances) are in millimeters.
void followBezierCurve(lib0xRobotCpp &robot, void* comm) {
    // --- Hardware Parameters (adjust these for your robot) ---
    const double distancePerCount = 1.0; // mm per encoder count
    const double axleLength_mm = 200.0;  // distance between wheels in mm
    const double wheelRadius_m = 0.05;   // wheel radius in meters

    // --- Get the starting global position and heading from the user ---
    double globalX, globalY, globalTheta;
    cout << "Enter starting global X position (mm): ";
    cin >> globalX;
    cout << "Enter starting global Y position (mm): ";
    cin >> globalY;
    cout << "Enter starting heading (radians): ";
    cin >> globalTheta;

    // Initialize previous encoder counts (for dead-reckoning)
    robot.getLeftMotorCount(comm, &prevLeftEncoder);
    robot.getRightMotorCount(comm, &prevRightEncoder);

    // Define the sampling resolution for the Bézier curve parameter t
    const double dt = 0.01;

    // Iterate over the curve parameter t from 0 to 1
    for (double t = 0.0; t <= 1.0; t += dt) {
        // Compute the target point on the Bézier curve (global coordinates in mm)
        Point target = computeBezierPoint(t, controlPoints);

        // Update the robot’s global position using encoder feedback
        updateRobotPosition(robot, comm, globalX, globalY, globalTheta,
                            distancePerCount, axleLength_mm);

        // Compute the control signals (v in m/s, w in rad/s) based on the current error
        double v, w;
        computeControlSignals(target.x, target.y, globalX, globalY, globalTheta, v, w);

        // Command the robot’s wheels with the computed velocities
        setWheelVelocities(robot, comm, v, w, wheelRadius_m, axleLength_mm);

        // Debug output for monitoring (optional)
        cout << "t: " << t 
             << " | Target: (" << target.x << ", " << target.y << ") mm"
             << " | Current: (" << globalX << ", " << globalY << ") mm"
             << " | Theta: " << globalTheta 
             << " | v: " << v << " m/s, w: " << w << " rad/s" << endl;

        // Allow time for the robot to move (adjust delay as needed)
        robot.DelaymSec(comm, 100);  // delay 100 milliseconds
    }

    // Stop the robot once the end of the curve is reached
    robot.stop(comm);
}

//---------------------------------------------------------
// Main Function
//---------------------------------------------------------

int main() {
    // Create an instance of the robot library
    lib0xRobotCpp robot;

    // Connect to the robot (adjust the port as needed, e.g., "COM3" on Windows or "/dev/ttyRobot" on Linux)
    void* comm = robot.connect_comm("/dev/ttyRobot");
    if (comm == nullptr) {
        cerr << "Error: Could not connect to robot." << endl;
        return -1;
    }

    // Initialize robot peripherals
    if (!robot.initPeripherals(comm)) {
        cerr << "Error: Could not initialize robot peripherals." << endl;
        robot.disconnect_comm(comm);
        return -1;
    }

    // Let the robot follow the Bézier curve using the given starting position and heading.
    followBezierCurve(robot, comm);

    // Disconnect from the robot when finished
    robot.disconnect_comm(comm);

    return 0;
}
