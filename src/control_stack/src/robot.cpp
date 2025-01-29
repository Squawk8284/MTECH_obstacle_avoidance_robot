// #include "robot.h"

// /**********************************************
//  *
//  *          INITIALISATION
//  *
//  *********************************************** */
// robot::robot(ros::NodeHandle nh) : robot_node_handle(nh) // initialising the protected mumber robot_node_handle with the nodehandle passed in the constructor
// {
//     // Parameter settings
//     robot_node_handle.param("robot_port", robot_serial_port, std::string(ROBOT_PORT));
//     ROS_INFO("RobotNode:: robot serial port as = %s", robot_serial_port.c_str());

//     robot_node_handle.param("imu_port", imu_serial_port, std::string(IMU_PORT));
//     ROS_INFO("RobotNode:: IMU serial port as = %s", imu_serial_port.c_str());

//     robot_node_handle.param("wheel_diameter", wheel_diameter, static_cast<double>(WHEEL_DIAMTER_IN_MM));
//     ROS_INFO("RobotNode:: Wheel Diamter as = %f", wheel_diameter);

//     robot_node_handle.param("axel_length", axel_length, static_cast<double>(AXEL_LENGTH_IN_MM));
//     ROS_INFO("RobotNode:: Axel Length as = %f", axel_length);

//     robot_node_handle.param("counts_per_rev", counts_per_rev, static_cast<int>(COUNTS_PER_REVOLUTION));
//     ROS_INFO("RobotNode:: Counts per revolution as = %d", counts_per_rev);

//     // Odom Setting
//     pose_pub = robot_node_handle.advertise<nav_msgs::Odometry>("odom", 1000);

//     // IMU Setting
//     imuPub = robot_node_handle.advertise<sensor_msgs::Imu>("imu", 50);

//     x_imu = 0;
//     y_imu = 1;
//     z_imu = 2;
//     gyro_angle = 0.0;
//     orientation_imu = 0;

//     set_imu_linear_acc_covarience();

//     set_imu_angular_vel_covarience();

//     set_imu_orientation_covarience();

//     // Dimension settings
//     distance_per_count = (wheel_diameter * 22.0 / 7.0) / counts_per_rev;
//     ROS_INFO("RobotNode:: Ditance per count = %f", distance_per_count);

//     // Odom Settings
//     leftCountPrev = 0;
//     rightCountPrev = 0;

//     velocity_x = 0;
//     velocity_y = 0;
//     velocity_theta = 0;

//     loopCount = 0;
//     updateFrequency = 10;
// }

// robot::~robot()
// {
//     /*
//      *   ROBOT DESTRUCTOR
//      */
//     RobotInt->stop(ROB_COM_HANDLE);            // stop robot motors
//     RobotInt->disconnect_comm(ROB_COM_HANDLE); // disconnect robot

//     /*
//      *   IMU DESTRUCTOR
//      */
//     IMUInt->disconnect_comm(IMU_COM_HANDLE); // disconnect IMU Port
// }

// /**********************************************
//  *
//  *          ROBOT INITIALISATION
//  *
//  *********************************************** */

// int robot::init()
// {
//     /*
//      *   ROBOT INITIALISATIONS
//      */
//     RobotInt = new lib0xRobotCpp();                                     // actually assigns the memory for the pointer
//     ROB_COM_HANDLE = RobotInt->connect_comm(robot_serial_port.c_str()); // connect to serial robot port
//     // Stop the motors
//     RobotInt->stop(ROB_COM_HANDLE);
//     // Reset Encoders
//     RobotInt->resetMotorEncoderCount(ROB_COM_HANDLE);

//     /*
//      *   IMU INITIALISATIONS
//      */
//     IMUInt = new lib0xRobotCpp();
//     IMU_COM_HANDLE = IMUInt->connect_comm(imu_serial_port.c_str()); // connect to serial IMU port

//     return 0;
// }

// /**********************************************
//  *
//  *          IMU
//  *
//  *********************************************** */

// bool robot::imu_init(void)
// {
//     IMUInt->imuInit(IMU_COM_HANDLE, gyroXYZ_Zero);
//     int16 gyroXYZ[3];
//     int32 gyroX = 0, gyroY = 0, gyroZ = 0;

//     for (int i = 0; i < 64; i++)
//     {
//         if (IMUInt->getGyroXYZ(IMU_COM_HANDLE, gyroXYZ) != 0)
//         {
//             gyroX += gyroXYZ[x_imu];
//             gyroY += gyroXYZ[y_imu];
//             gyroZ += gyroXYZ[z_imu];
//         }
//         else
//         {
//             return false;
//         }
//     }

//     gyroXYZ_Zero[x_imu] = (int16)(gyroX >> 6);
//     gyroXYZ_Zero[y_imu] = (int16)(gyroY >> 6);
//     gyroXYZ_Zero[z_imu] = (int16)(gyroZ >> 6);

//     return true;
// }

// bool robot::get_imu_data(sensor_msgs::Imu *imu_data)
// {
//     double orientation_angle = 0.0;
//     int16 accelerometerXYZ[3], gyroXYZ[3], magXYZ[3];
//     float accelerometerXYZ_g[3];
//     float magXYZ_gauss[3];

//     // initialize imu data
//     imu_data->linear_acceleration.x = 0.0;
//     imu_data->linear_acceleration.y = 0.0;
//     imu_data->linear_acceleration.z = 0.0;

//     if (IMUInt->getAccelerometerXYZ(IMU_COM_HANDLE, accelerometerXYZ) != 0)
//     {
//         // Acceleration (g) = (16 bit Raw data) * 0.004   for +/- 8g range
//         accelerometerXYZ_g[x_imu] = (accelerometerXYZ[x_imu] >> 4) * 0.004;
//         accelerometerXYZ_g[y_imu] = (accelerometerXYZ[y_imu] >> 4) * 0.004;
//         accelerometerXYZ_g[z_imu] = (accelerometerXYZ[z_imu] >> 4) * 0.004;

//         imu_data->linear_acceleration.x = accelerometerXYZ_g[x_imu] * 9.80665F;
//         imu_data->linear_acceleration.x = accelerometerXYZ_g[y_imu] * 9.80665F;
//         imu_data->linear_acceleration.x = accelerometerXYZ_g[z_imu] * 9.80665F;

//         // tilt compensated orientation requires acceleration data
//         if (IMUInt->getMagnetometerXYZ(IMU_COM_HANDLE, magXYZ) != 0)
//         {
//             magXYZ_gauss[x_imu] = magXYZ[x_imu] / 1100.0;
//             magXYZ_gauss[y_imu] = magXYZ[y_imu] / 1100.0;
//             magXYZ_gauss[z_imu] = magXYZ[z_imu] / 980.0;
//             orientation_angle = IMUInt->getTiltHeading(IMU_COM_HANDLE, magXYZ_gauss, accelerometerXYZ_g) * M_PI / 180.0;
//             imu_data->orientation = tf::createQuaternionMsgFromYaw(orientation_angle);
//         }
//         else
//             return false;
//     }
//     else
//         return false;

//     imu_data->angular_velocity.x = 0.0;
//     imu_data->angular_velocity.y = 0.0;
//     imu_data->angular_velocity.z = 0.0;

//     if (IMUInt->getGyroXYZ(IMU_COM_HANDLE, gyroXYZ) != 0)
//     {
//         imu_data->angular_velocity.x = (gyroXYZ[x_imu] - gyroXYZ_Zero[x_imu]) * 0.00175 * M_PI / 180.0;
//         imu_data->angular_velocity.y = (gyroXYZ[y_imu] - gyroXYZ_Zero[y_imu]) * 0.00175 * M_PI / 180.0;
//         imu_data->angular_velocity.z = (gyroXYZ[z_imu] - gyroXYZ_Zero[z_imu]) * 0.00175 * M_PI / 180.0;
//     }
//     else
//         return false;

//     return true;
// }

// void robot::set_imu_linear_acc_covarience()
// {
//     imu_data.linear_acceleration_covariance[0] = 0.001;
//     imu_data.linear_acceleration_covariance[1] = 0.0;
//     imu_data.linear_acceleration_covariance[2] = 0.0;
//     imu_data.linear_acceleration_covariance[3] = 0.0;
//     imu_data.linear_acceleration_covariance[4] = 0.001;
//     imu_data.linear_acceleration_covariance[5] = 0.0;
//     imu_data.linear_acceleration_covariance[6] = 0.0;
//     imu_data.linear_acceleration_covariance[7] = 0.0;
//     imu_data.linear_acceleration_covariance[8] = 0.001;
// }
// void robot::set_imu_angular_vel_covarience()
// {
//     imu_data.angular_velocity_covariance[0] = 0.5;
//     imu_data.angular_velocity_covariance[1] = 0.0;
//     imu_data.angular_velocity_covariance[2] = 0.0;
//     imu_data.angular_velocity_covariance[3] = 0.0;
//     imu_data.angular_velocity_covariance[4] = 0.5;
//     imu_data.angular_velocity_covariance[5] = 0.0;
//     imu_data.angular_velocity_covariance[6] = 0.0;
//     imu_data.angular_velocity_covariance[7] = 0.0;
//     imu_data.angular_velocity_covariance[8] = 0.5;
// }
// void robot::set_imu_orientation_covarience()
// {
//     imu_data.orientation_covariance[0] = 0.1;
//     imu_data.orientation_covariance[1] = 0.0;
//     imu_data.orientation_covariance[2] = 0.0;
//     imu_data.orientation_covariance[3] = 0.0;
//     imu_data.orientation_covariance[4] = 0.1;
//     imu_data.orientation_covariance[5] = 0.0;
//     imu_data.orientation_covariance[6] = 0.0;
//     imu_data.orientation_covariance[7] = 0.0;
//     imu_data.orientation_covariance[8] = 0.1;
// }

// /**********************************************
//  *
//  *          ODOMETRY
//  *
//  *********************************************** */
// void robot::get_position(nav_msgs::Odometry *position)
// {

//     // Getting left and right motor count
//     RobotInt->getLeftMotorCount(ROB_COM_HANDLE, &leftMotorCount);
//     RobotInt->getRightMotorCount(ROB_COM_HANDLE, &rightMotorCount);

//     // Find the incremental change
//     deltaLeftCount = leftMotorCount - leftCountPrev;
//     deltaRightCount = rightMotorCount - rightMotorCount;

//     // uses get_imu_data function to calcualte heading and orientation.
//     RobotInt->getDeltaPosition(ROB_COM_HANDLE, deltaLeftCount, deltaRightCount, theta, distance_per_count, axel_length, &deltaX, &deltaY, &deltaUpdate); 
//     x_vel += deltaX;
//     y_vel += deltaY;
//     theta += deltaTheta;
//     theta_Deg = (180.0 * 7.0 / 22.0) * theta;

//     position->pose.pose.position.x = x_vel / 1000;
//     position->pose.pose.position.y = y_vel / 1000;
//     tf::Quaternion quat = tf::createQuaternionFromYaw(theta);
//     tf::quaternionTFToMsg(quat, position->pose.pose.orientation);

//     setPoseCovariance(position);

//     velocity_x += deltaX;
//     velocity_y += deltaY;
//     velocity_theta += deltaTheta;

//     position->twist.twist.linear.x = RobotInt->getVelocityX(ROB_COM_HANDLE, deltaX, updateFrequency);
//     position->twist.twist.linear.y = RobotInt->getVelocityY(ROB_COM_HANDLE, deltaY, updateFrequency);
//     position->twist.twist.angular.z = RobotInt->getVelocityTheta(ROB_COM_HANDLE, deltaTheta, updateFrequency);
//     setTwistCovariance(position);
// }

// void robot::setPoseCovariance(nav_msgs::Odometry *position)
// {
//     position->pose.covariance[0] = 0.25;
//     position->pose.covariance[7] = 0.25;
//     position->pose.covariance[14] = 0.25;
//     position->pose.covariance[21] = 0.25;
//     position->pose.covariance[28] = 0.25;
//     position->pose.covariance[35] = 0.25;
// }

// void robot::setTwistCovariance(nav_msgs::Odometry *position)
// {
//     position->twist.covariance[0] = 0.04;
//     position->twist.covariance[7] = 0.04;
//     position->twist.covariance[14] = 0.04;
//     position->twist.covariance[21] = 0.09;
//     position->twist.covariance[28] = 0.09;
//     position->twist.covariance[35] = 0.09;
// }