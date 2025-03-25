

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.5
#define ANGULAR_VELOCITY 1.5

#define GET_ROBOT_DIRECTION 0
#define DRIVE_FORWARD 			1
#define RIGHT_TURN    			2
#define LEFT_TURN     			3

class deltadrive{

	public:					
					// Function prototypes
					deltadrive();
					~deltadrive();
					bool init();
					void controlloop();

	private:	
					// ROS node handler
					ros::NodeHandle nh_;
					
					// ROS topic publisher		
					ros::Publisher pub_cmd_vel;

					//ROS topic subscriber
					ros::Subscriber sub_laser_scan;
					ros::Subscriber sub_odom;

						
					// Variables
					double scan_data[3] = {0.0, 0.0, 0.0};					
					double yaw_angle_delta;
					double prev_yaw_angle_delta;
					double escape_range;
					double front_range_threshold;
					double side_range_threshold; 

					// Function prototypes
					void updatecommandVelocity(double linear, double angular);
					void laserScanmsgcallback(const sensor_msgs::LaserScan::ConstPtr &msg);		
					void odomMsgcallback(const nav_msgs::Odometry::ConstPtr &msg);			
};

