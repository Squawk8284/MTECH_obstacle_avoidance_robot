#ifndef __ROBOT__
#define __ROBOT__

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "0xRobotcpplib.h"
#include "robot_macros.h"

class robot
{
public:
    robot(ros::NodeHandle nh);
    virtual ~robot();

private:
    int init();
    void spin();
    void publish();

protected:
    ros::NodeHandle nh;
};

robot::robot(ros::NodeHandle nh) : nh(nh) // initialising the protected mumber nh with the nodehandle passed in the constructor
                                   {
                                       nh.param}

                                   robot::~robot()
{
}

#endif // __ROBOT__