#include <stdio.h>
#include <stdint.h>
#include <ros/ros.h>
#include "0xRobotcpplib.h"

#define ROS_NODE "RosRobotNode"

lib0xRobotCpp robot;
void *serial_port;

int main(int argc, char **argv)
{
    /* code */
    ros::init(argc, argv, ROS_NODE);
    ros::NodeHandle nh(std::string("~")); // This allows the node to publish topics under its nodespace which in this case is ROS_NODE/...

    return 0;
}
