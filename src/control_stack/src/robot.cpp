#include "robot.h"

robot::robot(ros::NodeHandle nh) : robot_node_handle(nh) // initialising the protected mumber robot_node_handle with the nodehandle passed in the constructor
{
    // Parameter settings
    robot_node_handle.param("robot_port", robot_serial_port, std::string(ROBOT_PORT));
    ROS_INFO("RobotNode:: robot serial port as = %s", robot_serial_port);

    robot_node_handle.param("imu_port", imu_serial_port, std::string(IMU_PORT));
    ROS_INFO("RobotNode:: IMU serial port as = %s", imu_serial_port);

    robot_node_handle.param("wheel_diameter", wheel_diameter, static_cast<double>(WHEEL_DIAMTER_IN_MM));
    ROS_INFO("RobotNode:: Wheel Diamter as = %f", wheel_diameter);

    robot_node_handle.param("axel_length", axel_length, static_cast<double>(AXEL_LENGTH_IN_MM));
    ROS_INFO("RobotNode:: Axel Length as = %f", axel_length);

    robot_node_handle.param("counts_per_rev", counts_per_rev, static_cast<int>(COUNTS_PER_REVOLUTION));
    ROS_INFO("RobotNode:: Counts per revolution as = %f", counts_per_rev);

    // IMU Setting
    imuPub = robot_node_handle.advertise<sensor_msgs::Imu>("robot_imu_handle", 50);

    x_imu = 0;
    y_imu = 1;
    z_imu = 2;
    gyro_angle = 0.0;
    orientation_imu = 0;

    imu_data.linear_acceleration_covariance[0] = 0.001;
    imu_data.linear_acceleration_covariance[1] = 0.0;
    imu_data.linear_acceleration_covariance[2] = 0.0;
    imu_data.linear_acceleration_covariance[3] = 0.0;
    imu_data.linear_acceleration_covariance[4] = 0.001;
    imu_data.linear_acceleration_covariance[5] = 0.0;
    imu_data.linear_acceleration_covariance[6] = 0.0;
    imu_data.linear_acceleration_covariance[7] = 0.0;
    imu_data.linear_acceleration_covariance[8] = 0.001;

    imu_data.angular_velocity_covariance[0] = 0.5;
    imu_data.angular_velocity_covariance[1] = 0.0;
    imu_data.angular_velocity_covariance[2] = 0.0;
    imu_data.angular_velocity_covariance[3] = 0.0;
    imu_data.angular_velocity_covariance[4] = 0.5;
    imu_data.angular_velocity_covariance[5] = 0.0;
    imu_data.angular_velocity_covariance[6] = 0.0;
    imu_data.angular_velocity_covariance[7] = 0.0;
    imu_data.angular_velocity_covariance[8] = 0.5;

    imu_data.orientation_covariance[0] = 0.1;
    imu_data.orientation_covariance[1] = 0.0;
    imu_data.orientation_covariance[2] = 0.0;
    imu_data.orientation_covariance[3] = 0.0;
    imu_data.orientation_covariance[4] = 0.1;
    imu_data.orientation_covariance[5] = 0.0;
    imu_data.orientation_covariance[6] = 0.0;
    imu_data.orientation_covariance[7] = 0.0;
    imu_data.orientation_covariance[8] = 0.1;

    // Dimension settings
    distance_per_count = (wheel_diameter * 22.0 / 7.0) / counts_per_rev;
    ROS_INFO("RobotNode:: Ditance per count = %f", distance_per_count);
}

robot::~robot()
{
}

bool robot::imu_init(void)
{
    int16 gyroXYZ[3];
    int32 gyroX = 0, gyroY = 0, gyroZ = 0;

    for (int i = 0; i < 64; i++)
    {
        if (RobotInt->getGyroXYZ(RobotInt->comm_handle, gyroXYZ) != 0)
        {
            gyroX += gyroXYZ[x_imu];
            gyroY += gyroXYZ[y_imu];
            gyroZ += gyroXYZ[z_imu];
        }
        else
        {
            return false;
        }
    }

    gyroXYZ_Zero[x_imu] = (int16)(gyroX >> 6);
    gyroXYZ_Zero[y_imu] = (int16)(gyroY >> 6);
    gyroXYZ_Zero[z_imu] = (int16)(gyroZ >> 6);

    return true;
}

bool robot::get_imu_data(sensor_msgs::Imu *imu_data)
{
    double orientation_angle = 0.0;
    int16 accelerometerXYZ[3], gyroXYZ[3], magXYZ[3];
    float accelerometerXYZ_g[3];
    float magXYZ_gauss[3];

    // initialize imu data
    imu_data->linear_acceleration.x = 0.0;
    imu_data->linear_acceleration.y = 0.0;
    imu_data->linear_acceleration.z = 0.0;

    if (RobotInt->getAccelerometerXYZ(RobotInt->comm_handle, accelerometerXYZ) != 0)
    {
        // Acceleration (g) = (16 bit Raw data) * 0.004   for +/- 8g range
        accelerometerXYZ_g[x_imu] = (accelerometerXYZ[x_imu] >> 4) * 0.004;
        accelerometerXYZ_g[y_imu] = (accelerometerXYZ[y_imu] >> 4) * 0.004;
        accelerometerXYZ_g[z_imu] = (accelerometerXYZ[z_imu] >> 4) * 0.004;

        imu_data->linear_acceleration.x = accelerometerXYZ_g[x_imu] * 9.80665F;
        imu_data->linear_acceleration.x = accelerometerXYZ_g[y_imu] * 9.80665F;
        imu_data->linear_acceleration.x = accelerometerXYZ_g[z_imu] * 9.80665F;

        // tilt compensated orientation requires acceleration data
        if (RobotInt->getMagnetometerXYZ(RobotInt->comm_handle, magXYZ) != 0)
        {
            magXYZ_gauss[x_imu] = magXYZ[x_imu] / 1100.0;
            magXYZ_gauss[y_imu] = magXYZ[y_imu] / 1100.0;
            magXYZ_gauss[z_imu] = magXYZ[z_imu] / 980.0;
            orientation_angle = RobotInt->getTiltHeading(RobotInt->comm_handle, magXYZ_gauss, accelerometerXYZ_g) * M_PI / 180.0;
            imu_data->orientation = tf::createQuaternionMsgFromYaw(orientation_angle);
        }
        else
            return false;
    }
    else
        return false;

    imu_data->angular_velocity.x = 0.0;
    imu_data->angular_velocity.y = 0.0;
    imu_data->angular_velocity.z = 0.0;

    if (RobotInt->getGyroXYZ(RobotInt->comm_handle, gyroXYZ) != 0)
    {
        imu_data->angular_velocity.x = (gyroXYZ[x_imu] - gyroXYZ_Zero[x_imu]) * 0.00175 * M_PI / 180.0;
        imu_data->angular_velocity.y = (gyroXYZ[y_imu] - gyroXYZ_Zero[y_imu]) * 0.00175 * M_PI / 180.0;
        imu_data->angular_velocity.z = (gyroXYZ[z_imu] - gyroXYZ_Zero[z_imu]) * 0.00175 * M_PI / 180.0;
    }
    else
        return false;

    return true;
}
