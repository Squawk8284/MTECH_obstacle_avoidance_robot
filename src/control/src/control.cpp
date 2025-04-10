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
#define AXEL_LENGTH_IN_MM (690)           // As per the hardware manual
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
double distancePerCount;

std::string cmd_vel_topic;
std::string odom_topic;

float start_x;
float start_y;
float start_theta;

Pose robotPose;

// ---------------------------------------------------------------------------
// Main Code
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{

    try
    {
        ros::init(argc, argv, "control");
        robotPort = createSerial("/dev/ttyRobot", 57600);

        ros::NodeHandle nh;
        ROS_INFO("Waiting for 8 seconds...");
        ros::Duration(8.0).sleep(); // Sleep for 2 seconds
        ROS_INFO("Starting.............\n\r");

        ros::param::param<std::string>("/cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
        ros::param::param<std::string>("/odom_topic", odom_topic, "/odom");

        ros::param::param<float>("/start_x", start_x, 0.0f);
        ros::param::param<float>("/start_y", start_y, 0.0f);
        ros::param::param<float>("/start_theta", start_theta, 0.0f);

        ros::Rate loopRate(10);

        ros::Subscriber sub = nh.subscribe(cmd_vel_topic, 10, cmdVelCallback);

        tf2_ros::TransformBroadcaster odom_broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        nav_msgs::Odometry odom_msg;

        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);

        init();

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
        clearSerial(robotPort); // Close serial port
    }

    return 0;
}
