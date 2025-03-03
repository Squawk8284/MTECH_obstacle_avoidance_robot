#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
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
    ros::Publisher cmd_vel_pub;
    
    Vector3d state;                 // [x, y, theta] in world frame
    vector<Vector2d> control_points; // Bézier curve control points (in meters)
    Vector3d ref_state;             // Desired state: [x_d, y_d, theta_d]
    Vector2d control;               // [v, w] where v is forward velocity, w is angular velocity
    double t;                     // Current best parameter along the curve
    bool reached_end;
    
    // Odometry callback: updates the current state and computes/publishes control commands
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = tf::getYaw(msg->pose.pose.orientation);
        state << x, y, theta;
        
        if (!reached_end) {
            updateReferenceState();
            computeControl();
        } else {
            control.setZero();
        }
        publishControl();
    }
    
    // Update the reference state along the Bézier curve using a lookahead based on the closest point.
    void updateReferenceState() {
        // Find the closest parameter on the curve to the current position.
        double t_closest = findClosestT(state.head<2>());
        double lookahead = 0.05;  // Lookahead offset in parameter space (adjust as needed)
        double t_ref = min(t_closest + lookahead, 1.0);
        t = t_ref;  // Update current t

        Vector2d point = computeBezierPoint(t_ref);
        ref_state(0) = point(0);
        ref_state(1) = point(1);
        // Desired heading: angle from current position to the reference point.
        ref_state(2) = atan2(point(1) - state(1), point(0) - state(0));
        
        if (t_ref >= 1.0)
            reached_end = true;
    }
    
    // Compute the closest t (in [0,1]) on the Bézier curve to the given 2D position.
    double findClosestT(const Vector2d &pos) {
        double best_t = 0.0;
        double min_dist = numeric_limits<double>::max();
        // Use a fine resolution to search along the curve.
        for (double t_candidate = 0.0; t_candidate <= 1.0; t_candidate += 0.001) {
            Vector2d pt = computeBezierPoint(t_candidate);
            double dist = (pt - pos).norm();
            if (dist < min_dist) {
                min_dist = dist;
                best_t = t_candidate;
            }
        }
        return best_t;
    }
    
    // Compute a point on the Bézier curve for parameter t using Bernstein polynomials.
    Vector2d computeBezierPoint(double t) {
        int n = control_points.size() - 1;
        Vector2d point(0, 0);
        for (int i = 0; i <= n; ++i) {
            double binomial_coeff = factorial(n) / (factorial(i) * factorial(n - i));
            double bernstein = binomial_coeff * pow(t, i) * pow(1 - t, n - i);
            point += bernstein * control_points[i];
        }
        return point;
    }
    
    // Simple recursive factorial function.
    int factorial(int num) {
        return (num <= 1) ? 1 : num * factorial(num - 1);
    }
    
    // Compute control inputs.
    // Here, the control is computed by first calculating the desired velocity vector in the world frame (from current position to the reference)
    // and then projecting that onto the robot’s heading (since a differential-drive robot can only command forward velocity).
    void computeControl() {
        // Compute position error in world frame
        Vector2d pos_error = ref_state.head<2>() - state.head<2>();
        // Desired velocity vector (proportional to error)
        Vector2d v_desired = pos_error;
        // Robot’s heading vector in world frame
        Vector2d heading(cos(state(2)), sin(state(2)));
        // Project desired velocity onto heading
        double v_forward = v_desired.dot(heading);
        
        // Compute angular error (difference between desired heading and current heading)
        double ang_error = ref_state(2) - state(2);
        ang_error = atan2(sin(ang_error), cos(ang_error));  // Normalize to [-pi, pi]
        
        // Set control inputs
        control(0) = v_forward;
        control(1) = ang_error;  // Optionally scale this gain if needed.
    }
    
    // Publish control commands to /cmd_vel.
    void publishControl() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = control(0);
        cmd.angular.z = control(1);
        cmd_vel_pub.publish(cmd);
    }
    
public:
    GMPCController() : t(0.0), reached_end(false) {
        odom_sub = nh.subscribe("/odom", 10, &GMPCController::odomCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // Define the Bézier curve control points (in meters)
        control_points = { {0.6, 0.6}, {4.37, 0.667}, {4.49, 1.226}, {6.9, 4.2} };
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
