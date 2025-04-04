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

// ---------------------------------------------------------------------------
// Robot Parameters
// ---------------------------------------------------------------------------
#define AXEL_LENGTH_IN_MM (590)           // As per the hardware manual
#define WHEEL_DIA_IN_MM (260)             // As per the hardware manual
#define TIRE_RUBBER_THREAD_DIA_IN_MM (80) // As per the hardware manual

// ---------------------------------------------------------------------------
// Libraries and objects
// ---------------------------------------------------------------------------
// User Defined Library
#include <ros_callbacks.hpp>

// Global Objects
serial::Serial *robotPort = nullptr;
serial::Serial *imuPort = nullptr;
float axel_length_m;
float wheel_dia_m;
uint16_t CountsPerWheelRevolution;
uint16_t WheelRevPerCounts;

Pose robotPose;

// ---------------------------------------------------------------------------
// Main Code
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{

    try
    {
        robotPort = createSerial("/dev/ttyRobot", 57600);
        init();
        ros::init(argc, argv, "control");
        ros::NodeHandle nh;

        ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);
        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
        tf2_ros::TransformBroadcaster odom_broadcaster;

        ROS_INFO("Waiting for 2 seconds...");
        ros::Duration(2.0).sleep(); // Sleep for 2 seconds
        ROS_INFO("Resuming execution!");

        while (ros::ok())
        {
            UpdateOdometry(odom_pub, odom_broadcaster);
            ros::spinOnce();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        clearSerial(robotPort);
    }

    return 0;
}
