#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <serial/serial.h>
#include <vector>
#include <sstream>
#include <string>
#include <geometry_msgs/Twist.h>
#include "0xRobotcpplib.h"


// ***************************************************
//          MACROS
// ***************************************************

//##### ROBOT MACROS #####//



//##### IMU MACROS ####//
#define DEG_TO_RAD                              (M_PI/180.0)
#define ACCEL_CONVERSION                        (9.806/256.0)
#define GYRO_CONVERSION                         (M_PI/180.0)


// ***************************************************
//          GLOBAL VARIABLES
// ***************************************************

// Global Variables for velocity commands
double g_linear_vel = 0.0;
double g_angular_vel = 0.0;

// Robot Odometry State
double g_x,g_y,g_theta; // x,y and theta in rad


// ***************************************************
//          CALL BACK FUNCTIONS
// ***************************************************

void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    g_linear_vel = std::sqrt((msg->linear.x * msg->linear.x) + (msg->linear.y * msg->linear.y)); //in m/s

    g_angular_vel = msg->angular.z; // in rad/s
}

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf2::Quaternion q;
    q.setX(msg->pose.orientation.x);
    
}

int main()
{
    return 0;
}