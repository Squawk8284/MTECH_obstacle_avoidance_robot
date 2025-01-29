// #ifndef __ROBOT__
// #define __ROBOT__

// #include <stdio.h>

// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "geometry_msgs/Twist.h"
// #include "nav_msgs/Odometry.h"
// #include "sensor_msgs/Imu.h"
// #include "tf/tf.h"
// #include "tf/transform_listener.h"
// #include "tf/transform_broadcaster.h"
// #include "tf/transform_datatypes.h"

// #include "0xRobotcpplib.h"
// #include "robot_macros.h"

// class robot
// {
// public:
//     robot(ros::NodeHandle nh);
//     virtual ~robot();

// private:
//     int init();
//     void spin();
//     void publish();

// protected:
//     /******* ROBOT INSTANCE *******/
//     lib0xRobotCpp *RobotInt;
//     lib0xRobotCpp *IMUInt;
//     ros::NodeHandle robot_node_handle;

//     uint32 loopCount;
//     uint8 updateFrequency;


//     /******* PORT SETTINGS *******/
//     std::string robot_serial_port;
//     std::string imu_serial_port;

//     /******* DIMENSTION SETTINGS *******/
//     double wheel_diameter;
//     int counts_per_rev;
//     double axel_length;
//     double distance_per_count;

//     /******* IMU SETTINGS *******/
//     sensor_msgs::Imu imu_data;
//     ros::Publisher imuPub;

//     uint8 x_imu,y_imu,z_imu;
//     int16 gyroXYZ_Zero[3];
//     float gyro_angle;
//     double orientation_imu;

//     bool imu_init(void);
//     bool get_imu_data(sensor_msgs::Imu *imu_data);

//     void set_imu_linear_acc_covarience();
//     void set_imu_angular_vel_covarience();
//     void set_imu_orientation_covarience();


//     /******* ODOM SETTINGS *******/
//     nav_msgs::Odometry position;
//     ros::Publisher pose_pub;

//     int32 leftMotorCount, rightMotorCount, leftCountPrev, rightCountPrev, deltaLeftCount, deltaRightCount;
//     double velocity_x, velocity_y,velocity_theta;
//     double x_vel, y_vel, deltaX, deltaY, deltaUpdate;
//     double theta, deltaTheta, theta_Deg;

//     void get_position(nav_msgs::Odometry *position);
//     void setPoseCovariance(nav_msgs::Odometry *position);
//     void setTwistCovariance(nav_msgs::Odometry *position);





// };

// #endif // __ROBOT__