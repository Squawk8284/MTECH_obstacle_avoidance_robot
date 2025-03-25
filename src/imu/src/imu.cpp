#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <tf/tf.h>
#include <serial/serial.h>
#include <vector>
#include <sstream>
#include <string>

#define DEG_TO_RAD (M_PI/180.0)
#define ACCEL_CONVERSION (9.806/256.0)
#define GYRO_CONVERSION (M_PI/180.0)

class RazorIMU {
public:
    RazorIMU(ros::NodeHandle& nh) : nh_(nh) {
        nh_.param<std::string>("port", port_, "/dev/ttyIMU");
        nh_.param<std::string>("topic", topic_, "imu");
        nh_.param<std::string>("frame_id", frame_id_, "base_imu_link");
        nh_.param("imu_yaw_calibration", imu_yaw_calibration_, 0.0);

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(topic_, 1);
        diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);

        try {
            serial_.setPort(port_);
            serial_.setBaudrate(57600);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
        } catch (serial::IOException& e) {
            ROS_ERROR("Unable to open serial port: %s", port_.c_str());
            ros::shutdown();
        }
        if (serial_.isOpen()) {
            ROS_INFO("Connected to IMU on port %s", port_.c_str());
        }

        ros::Duration(5.0).sleep(); // Allow IMU to initialize
        serial_.write("#o0\n"); // Stop any ongoing stream
        ros::Duration(0.1).sleep();
        serial_.write("#ox\n"); // Set output mode to angles, accel, gyro
        ros::Duration(0.1).sleep();
        serial_.write("#f\n"); // Request output data string
        ros::Duration(0.1).sleep();
    }

    void readIMUData() {
        std::string line;
        while (ros::ok()) {
            try {
                serial_.write("#f\n"); // Request output data string
                ros::Duration(0.1).sleep();
                line = serial_.readline(256, "\n");
                // ROS_INFO("Raw IMU Data: %s", line.c_str()); // Debugging raw data
                if (line.find("#YPRAG=") != std::string::npos) { // Adjusted for correct prefix
                    parseIMUData(line);
                } else {
                    ROS_WARN("Bad IMU data or sync issue");
                }
            } catch (serial::IOException& e) {
                ROS_ERROR("Serial communication error");
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_, diag_pub_;
    serial::Serial serial_;
    std::string port_, topic_, frame_id_;
    double imu_yaw_calibration_;
    unsigned int seq_ = 0;

    void parseIMUData(const std::string& data) {
        std::vector<std::string> tokens;
        std::stringstream ss(data.substr(7)); // Remove "#YPRAG="
        std::string token;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        if (tokens.size() != 9) return; // Expecting Yaw, Pitch, Roll, Accel X/Y/Z, Gyro X/Y/Z

        double yaw = -std::stod(tokens[0]) + imu_yaw_calibration_;
        double pitch = -std::stod(tokens[1]) * DEG_TO_RAD;
        double roll = std::stod(tokens[2]) * DEG_TO_RAD;

        tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = frame_id_;
        imu_msg.header.seq = seq_++;
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();

        imu_msg.linear_acceleration.x = std::stod(tokens[3]) * ACCEL_CONVERSION;
        imu_msg.linear_acceleration.y = std::stod(tokens[4]) * ACCEL_CONVERSION;
        imu_msg.linear_acceleration.z = std::stod(tokens[5]) * ACCEL_CONVERSION;

        imu_msg.angular_velocity.x = std::stod(tokens[6]) * GYRO_CONVERSION;
        imu_msg.angular_velocity.y = std::stod(tokens[7]) * GYRO_CONVERSION;
        imu_msg.angular_velocity.z = std::stod(tokens[8]) * GYRO_CONVERSION;

        // Apply covariance values
        for (int i = 0; i < 9; ++i) {
            imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.5 : 0.0;
            imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.001 : 0.0;
            imu_msg.orientation_covariance[i] = (i % 4 == 0) ? 0.1 : 0.0;
        }

        imu_pub_.publish(imu_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "razor_imu_node");
    ros::NodeHandle nh;
    RazorIMU imu(nh);
    imu.readIMUData();
    return 0;
}
