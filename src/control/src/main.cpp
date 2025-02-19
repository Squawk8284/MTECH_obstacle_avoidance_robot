#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "0xRobotcpplib.h"
#include <cmath>


// ***************************************************
//          MACROS
// ***************************************************
#define AXEL_LENGTH_IN_M    (0.7)   // in m
#define AXEL_LENGTH_IN_MM   (AXEL_LENGTH_IN_M*1000)   // in mm
#define WHEEL_RADIUS_IN_M   (0.13)  // in m
#define ENCODER_TICKS       (3840)  // ticks per revolution


// ***************************************************
//          GLOBAL VARIABLES
// ***************************************************

// Global Variables for velocity commands
double g_linear_cmd = 0.0;
double g_angular_cmd = 0.0;

// Robot Odometry State
double g_x =0.6, g_y = 0.6, g_theta =0; // x,y in m; theta in rad 

//Time tracking for odometry updates
ros::Time LastTime;

// ***************************************************
//          CALL BACK FUNCTIONS
// ***************************************************

void cmdVelocityCallback(const std_msgs::Float64::ConstPtr &msg)
{
    g_linear_cmd = msg->data;
}

void cmdAngularVelocityCallback(const std_msgs::Float64::ConstPtr &msg)
{
    g_angular_cmd = msg->data;
}


int main(int argc, char **argv)
{

    // Initialise the ros nodes
    ros::init(argc,argv,"control_node");
    ros::NodeHandle nh("~");

    // Get port parameter
    std::string port;
    nh.param<std::string>("port", port, "/dev/ttyRobot");

    //Initialise the Robot
    FireBirdcpp robot;
    void *commHandle = robot.connect_comm(port.c_str());
    if(!commHandle)
    {
        ROS_ERROR("Failed to connect to port %s", port.c_str());
        return 1;
    }

    // Set Closed loop Control
    if (!robot.setMode(commHandle, 1)) {
        ROS_ERROR("Failed to set motion control mode to closed-loop velocity control!");
        return 1;
    }

    if (!robot.setWheelDiameter_mm(commHandle, (WHEEL_RADIUS_IN_M*2*1000)))
    {
        ROS_ERROR("Failed to set wheel diameter!");
    }
  
    if (!robot.setRobotAxlelength_mm(commHandle, (AXEL_LENGTH_IN_M*1000)))
    {
        ROS_ERROR("Failed to set axle length!");
    }
    
    float axelLength;
    if (!robot.getRobotAxlelength_mm(commHandle,&axelLength))
    {
        ROS_ERROR("Failed to get axle length!");
    }
    



    float distancepercount = (M_PI * WHEEL_RADIUS_IN_M*2)/ENCODER_TICKS;
    int32 dLeft, dRight,dtheta;

    robot.stop(commHandle);
    robot.resetMotorEncoderCount(commHandle);
    robot.setSafetyTimeout(commHandle, 0);
    robot.setSafety(commHandle, 0);

    ROS_INFO("Connected to Robot on %s", port.c_str());

    // Subscribers for velocity commands
    ros::Subscriber sub_linear = nh.subscribe("/cmd_vel", 10, cmdVelocityCallback);
    ros::Subscriber sub_angular = nh.subscribe("/cmd_angular", 10, cmdAngularVelocityCallback);

    // Publishers for current velocities and odometry
    ros::Publisher pub_current_linear = nh.advertise<std_msgs::Float64>("/current_linear_velocity", 10);
    ros::Publisher pub_current_angular = nh.advertise<std_msgs::Float64>("/current_angular_velocity", 10);
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    tf::TransformBroadcaster odom_broadcaster;

    // Set loop rate
    ros::Rate loop_rate(10);  // 10 Hz
    LastTime = ros::Time::now();

    while(ros::ok())
    {
        ros::spinOnce();

        // Set linear and angular velocity
        if (!robot.setLinearVelocity_meterspersec(commHandle, static_cast<float>(g_linear_cmd))) {
            ROS_ERROR("Failed to set linear velocity");
        }

        if (!robot.setRobotAngularVelocityRadianpersec(commHandle, static_cast<float>(g_angular_cmd))) {
            ROS_ERROR("Failed to set angular velocity");
        }

        // Ensure the velocity takes effect
        if (!robot.forward(commHandle)) {
            ROS_ERROR("Failed to call forward()");
        }

        // Publish current velocities
        std_msgs::Float64 current_linear_msg;
        current_linear_msg.data = g_linear_cmd;
        pub_current_linear.publish(current_linear_msg);

        std_msgs::Float64 current_angular_msg;
        current_angular_msg.data = g_angular_cmd;
        pub_current_angular.publish(current_angular_msg);

        // === Odometry Update ===
        double delta_x = 0.0, delta_y = 0.0, delta_theta = 0.0;
        if(!robot.getLeftMotorCount(commHandle, &dLeft))
        {
            ROS_INFO("Failed to get Left Motor Count");
        }

        if(!robot.getRightMotorCount(commHandle, &dRight))
        {
            ROS_INFO("Failed to get Left Motor Count");
        }
        
        dtheta = (dLeft-dRight)*(WHEEL_RADIUS_IN_M/AXEL_LENGTH_IN_M);

        if (!robot.getDeltaPosition(commHandle, dLeft,dRight,dtheta, distancepercount, (AXEL_LENGTH_IN_MM),&delta_x, &delta_y, &delta_theta)) 
        {
            ROS_ERROR("Failed to get delta position");
        }

        // Update global odometry state
        g_x += delta_x * cos(g_theta) - delta_y * sin(g_theta);
        g_y += delta_x * sin(g_theta) + delta_y * cos(g_theta);
        g_theta += delta_theta;

        // Normalize theta to [-π, π]
        g_theta = atan2(sin(g_theta), cos(g_theta));
        // Publish odometry message
        ros::Time current_time = ros::Time::now();
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Set position
        odom_msg.pose.pose.position.x = g_x;
        odom_msg.pose.pose.position.y = g_y;
        odom_msg.pose.pose.position.z = 0.0;

        // Convert theta to quaternion
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(g_theta);
        odom_msg.pose.pose.orientation = odom_quat;

        // Set linear and angular velocity in odometry message
        odom_msg.twist.twist.linear.x = g_linear_cmd;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = g_angular_cmd;

        pub_odom.publish(odom_msg);

        // Publish transform for visualization
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = g_x;
        odom_tf.transform.translation.y = g_y;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_tf);

        loop_rate.sleep();

    }

    // Disconnect on shutdown
    if (!robot.disconnect_comm(commHandle)) {
        ROS_WARN("Failed to disconnect from %s", port.c_str());
    } else {
        ROS_INFO("Disconnected from %s", port.c_str());
    }


    return 0;
}