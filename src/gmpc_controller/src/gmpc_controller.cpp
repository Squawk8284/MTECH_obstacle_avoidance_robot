#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

using namespace Eigen;
using namespace std;

class GMPCController {
private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Subscriber path_sub;
    ros::Publisher cmd_vel_pub;
    
    Vector3d state;                 // [x, y, theta] in world frame
    nav_msgs::Path current_path;    // Received path message
    Vector3d ref_state;             // Desired state: [x_d, y_d, theta_d]
    Vector2d control;               // [v, w] where v is forward velocity, w is angular velocity
    bool reached_end;
    
    // Odometry callback: updates the current state and computes/publishes control commands.
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        q.normalize();
        double theta = tf::getYaw(q);
        state << x, y, theta;
        
        if (!reached_end) {
            updateReferenceState();
            computeControl();
        } else {
            control.setZero();
        }
        publishControl();
    }
    
    // Path callback: updates the current path received during runtime.
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        reached_end = false;
        current_path = *msg;
        // Normalize quaternions in the path to prevent errors
        for (auto &pose : current_path.poses) {
            tf::Quaternion q(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            );
            q.normalize();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
        }
    }
    
    // Update the reference state along the received path using a lookahead strategy.
    void updateReferenceState() {
        if (current_path.poses.empty()) return; // Avoid accessing empty path
        
        // Find the closest point in the received path to the current position.
        size_t closest_idx = findClosestPoint(state.head<2>());
        // Lookahead: take a point 5 indices ahead (or the last point if near the end)
        size_t lookahead_idx = min(closest_idx + 5, current_path.poses.size() - 1);
        
        ref_state(0) = current_path.poses[lookahead_idx].pose.position.x;
        ref_state(1) = current_path.poses[lookahead_idx].pose.position.y;
        ref_state(2) = tf::getYaw(current_path.poses[lookahead_idx].pose.orientation);
        
        // Check if reached the end of the path.
        if (closest_idx >= current_path.poses.size() - 1 &&
            (state.head<2>() - ref_state.head<2>()).norm() < 0.05) {
            reached_end = true;
        }
    }
    
    // Compute the closest point index in the path to the given position.
    size_t findClosestPoint(const Vector2d &pos) {
        size_t best_idx = 0;
        double min_dist = numeric_limits<double>::max();
        
        for (size_t i = 0; i < current_path.poses.size(); ++i) {
            double x = current_path.poses[i].pose.position.x;
            double y = current_path.poses[i].pose.position.y;
            Vector2d pt(x, y);
            double dist = (pt - pos).norm();
            if (dist < min_dist) {
                min_dist = dist;
                best_idx = i;
            }
        }
        return best_idx;
    }
    
    // Compute control inputs using a geometric approach.
    void computeControl() {
        // Compute the error vector from the robot to the reference point.
        Vector2d pos_error = ref_state.head<2>() - state.head<2>();
        double distance = pos_error.norm();
        
        // Compute the angle from the robot to the reference point.
        double angle_to_goal = atan2(pos_error(1), pos_error(0));
        // Compute heading error (difference between desired angle and current heading)
        double heading_error = angle_to_goal - state(2);
        heading_error = atan2(sin(heading_error), cos(heading_error)); // Normalize error
        
        // Control gains (tweak these as needed)
        double k_v = 0.5; // Linear velocity gain
        double k_w = 1.0; // Angular velocity gain
        
        // Compute forward velocity scaled by the cosine of the heading error.
        // This ensures that if the goal is behind the robot (cos(heading_error) < 0),
        // the robot does not drive backwards.
        double v_forward = k_v * distance * cos(heading_error);
        if(cos(heading_error) < 0) {
            v_forward = 0;
        }
        
        double w = k_w * heading_error;
        
        control(0) = v_forward;
        control(1) = w;
    }
    
    // Publish control commands to /cmd_vel.
    void publishControl() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = control(0);
        cmd.angular.z = control(1);
        cmd_vel_pub.publish(cmd);
    }
    
public:
    GMPCController() : reached_end(false) {
        odom_sub = nh.subscribe("/odom", 10, &GMPCController::odomCallback, this);
        path_sub = nh.subscribe("/path_topic", 10, &GMPCController::pathCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }
    
    void run() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gmpc_controller");
    GMPCController controller;
    controller.run();
    return 0;
}
