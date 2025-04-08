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
double WheelRevPerCounts;

Pose robotPose;

// ---------------------------------------------------------------------------
// Main Code
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{

    ros::init(argc, argv, "control");
    try
    {
        robotPort = createSerial("/dev/ttyRobot", 57600);
        init();
        ros::NodeHandle nh;
        ros::Rate loopRate(50);
        ros::Subscriber sub = nh.subscribe("/cmd_vel", 50, cmdVelCallback);
        tf2_ros::TransformBroadcaster odom_broadcaster;
        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

        geometry_msgs::TransformStamped odom_trans;
        nav_msgs::Odometry odom_msg;
        ROS_INFO("Waiting for 2 seconds...");
        ros::Duration(2.0).sleep(); // Sleep for 2 seconds
        ROS_INFO("Resuming execution!");

        while (ros::ok())
        {
            UpdateOdometry(odom_trans, odom_msg);
            odom_broadcaster.sendTransform(odom_trans);
            odom_pub.publish(odom_msg);
            ros::spinOnce();
            loopRate.sleep();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        clearSerial(robotPort);
    }

    return 0;
}
