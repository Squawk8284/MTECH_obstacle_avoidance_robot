
#include <delta_drive.h>

//Constructor
deltadrive::deltadrive(){

	ROS_INFO("Delta drive Node Init");
	ROS_ASSERT(init());
}

//Deconstructor
deltadrive::~deltadrive(){

	ROS_INFO("exiting!!!!!!!!");
	updatecommandVelocity(0.0, 0.0);
	ros::shutdown();
}


bool deltadrive::init(){

	escape_range = 10 * (M_PI / 180);
	yaw_angle_delta = 0.0;
	prev_yaw_angle_delta = 0.0;
	front_range_threshold = 1.1;
	side_range_threshold = 1.0;	

	pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/delta_velocity_controller/cmd_vel", 10);	
	sub_laser_scan = nh_.subscribe("/front/scan", 10, &deltadrive::laserScanmsgcallback, this);
	sub_odom = nh_.subscribe("/delta_velocity_controller/odom", 10, &deltadrive::odomMsgcallback, this);
	return true;
}


void deltadrive::updatecommandVelocity(double linear, double angular){

	geometry_msgs::Twist msg;

	msg.linear.x = linear;
	msg.angular.z = angular;

	pub_cmd_vel.publish(msg);
}

void deltadrive::laserScanmsgcallback(const sensor_msgs::LaserScan::ConstPtr &msg){

	uint16_t scan_angle[3] = {0, 30, 330};
	for(uint8_t index = 0; index < 3; index++){
		
		if(std::isinf(msg->ranges.at(scan_angle[index])))
			scan_data[index] = msg->range_max;
		else
			scan_data[index] = msg->ranges.at(scan_angle[index]);

		//ROS_INFO("ranges at %d degree = %f\n", scan_angle[index], msg->ranges.at(scan_angle[index]));
	}

}

void deltadrive::odomMsgcallback(const nav_msgs::Odometry::ConstPtr &msg){

	double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);

	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

	yaw_angle_delta = atan2(siny, cosy);
	ROS_INFO("angle = %f\n", yaw_angle_delta);
}

void deltadrive::controlloop(void){

	static uint8_t robot_state = GET_ROBOT_DIRECTION;

	switch(robot_state){
		
		case GET_ROBOT_DIRECTION: 
					if(scan_data[CENTER] > front_range_threshold){
					
						if(scan_data[LEFT] < side_range_threshold){
			
							prev_yaw_angle_delta = yaw_angle_delta;
							robot_state = RIGHT_TURN;
						}
						else if(scan_data[RIGHT] < side_range_threshold){
			
							prev_yaw_angle_delta = yaw_angle_delta;
							robot_state = LEFT_TURN;
						}
						else
							robot_state = DRIVE_FORWARD;
					}
					
					if(scan_data[0] < front_range_threshold){
			
						prev_yaw_angle_delta = yaw_angle_delta;
						robot_state = RIGHT_TURN;
					}
					break;

		case DRIVE_FORWARD: 
					updatecommandVelocity(LINEAR_VELOCITY, 0.0);
					robot_state = GET_ROBOT_DIRECTION;
					break;

		case RIGHT_TURN: 
					if(fabs(prev_yaw_angle_delta - yaw_angle_delta) >= escape_range)
						robot_state = GET_ROBOT_DIRECTION;
					else
						updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
					
					break;

		case LEFT_TURN:
					if(fabs(prev_yaw_angle_delta - yaw_angle_delta) >= escape_range)
						robot_state = GET_ROBOT_DIRECTION;
					else
						updatecommandVelocity(0.0, ANGULAR_VELOCITY);
					
					break;

		default:
					robot_state = GET_ROBOT_DIRECTION;
					break;
	}
}


/*-----------------------------------------------------------------------------
		Main Function		
------------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "delta_drive");

	deltadrive delta_drive;

	ros::Rate loop_rate(125);
	
	while(ros::ok()){
		
		delta_drive.controlloop();
		ros::spinOnce();	
		loop_rate.sleep();
	}

  return 0;
}
