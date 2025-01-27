#ifndef __ROBOT__
#define __ROBOT__

#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"

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
    /******* ROBOT INSTANCE *******/
    lib0xRobotCpp *RobotInt;
    lib0xRobotCpp *IMUInt;
    ros::NodeHandle robot_node_handle;

    /******* PORT SETTINGS *******/
    std::string robot_serial_port;
    std::string imu_serial_port;

    /******* DIMENSTION SETTINGS *******/
    double wheel_diameter;
    int counts_per_rev;
    double axel_length;
    double distance_per_count;

    /******* IMU SETTINGS *******/
    sensor_msgs::Imu imu_data;
    ros::Publisher imuPub;

    uint8 x_imu,y_imu,z_imu;
    int16 gyroXYZ_Zero[3];
    float gyro_angle;
    double orientation_imu;

    bool imu_init(void);
    bool get_imu_data(sensor_msgs::Imu *imu_data);
    float get_imu_heading(int16 *magValue);
    float get_tilt_heading(float *magValue, float *accelValue);







};

#endif // __ROBOT__