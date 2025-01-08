#include "ros/ros.h"
#include <iostream>
#include "0xRobotcpplib.h" // Ensure this is in the include path

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "robot_movement_node");
    ros::NodeHandle nh;

    // Log the start of the node
    ROS_INFO("Starting robot movement node...");

    // Create an instance of the robot control class
    lib0xRobotCpp robot;

    // Connect to the robot on /dev/ttyUSB1
    void* hSerial = robot.connect_comm("/dev/ttyUSB1");
    if (hSerial == nullptr) {
        ROS_ERROR("Failed to connect to the robot on /dev/ttyUSB1!");
        return -1;
    }
    ROS_INFO("Successfully connected to the robot.");

    robot.stop(hSerial);
    robot.resetMotorEncoderCount(hSerial);
    robot.setAcceleration(hSerial, 4);

    robot.setLinearVelocity_meterspersec(hSerial, 0.250);

    robot.setSafetyTimeout(hSerial, 0);
    robot.setSafety(hSerial, 0);
    // robot.buzzerOff(hSerial);

    // Move forward
    if (!robot.forward(hSerial)) {
        ROS_ERROR("Failed to move forward!");
    } else {
        ROS_INFO("Moving forward...");
        ros::Duration(5.0).sleep(); // Move forward for 5 seconds
    }

    // robot.DelaymSec(hSerial, 5000);

     // Stop the robot
    if (!robot.stop(hSerial)) {
        ROS_ERROR("Failed to stop the robot!");
    } else {
        ROS_INFO("Robot stopped.");
    }

    ros::Duration(2.0).sleep();

    if (!robot.backward(hSerial)) {
        ROS_ERROR("Failed to move forward!");
    } else {
        ROS_INFO("Moving backward...");
        ros::Duration(5.0).sleep(); // Move forward for 5 seconds
    }


    // Stop the robot
    if (!robot.stop(hSerial)) {
        ROS_ERROR("Failed to stop the robot!");
    } else {
        ROS_INFO("Robot stopped.");
    }

    // Disconnect the robot
    if (!robot.disconnect_comm(hSerial)) {
        ROS_ERROR("Failed to disconnect from the robot!");
    } else {
        ROS_INFO("Disconnected from the robot.");
    }

    return 0;
}
