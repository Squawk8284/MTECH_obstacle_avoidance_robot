#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <qpOASES.hpp>
#include <vector>
#include <limits>
#include <cmath>

using namespace qpOASES;
using namespace std;

class GMPCController {
private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Subscriber path_sub;
    ros::Publisher cmd_vel_pub;
    
    // Robot state: [x, y, theta]
    Eigen::Vector3d state;
    // Received reference trajectory as a path message
    nav_msgs::Path current_path;
    // Flag to check if a valid path is available
    bool path_available;
    
    // MPC parameters
    int T;           // Prediction horizon (number of steps)
    double delta_t;  // Discretization time step
    Eigen::Matrix3d Q;      // State penalty (for intermediate steps)
    Eigen::Matrix3d Qf;     // Terminal state penalty
    Eigen::Matrix2d Rm;     // Control penalty weight
    Eigen::Vector2d u_min;  // Lower bound on control input
    Eigen::Vector2d u_max;  // Upper bound on control input
    
    // For unicycle kinematics, constant input map: B = C(0) = [ [1, 0]; [0, 0]; [0, 1] ]
    Eigen::Matrix<double, 3, 2> B_const;
    
    // Desired velocity (zeta_d) and corresponding adm operator.
    Eigen::Vector3d zeta_d; // Desired body velocity: [v, 0, omega]
    
    // Reference state extracted from the path (lookahead)
    Eigen::Vector3d ref_state;
    // Error state in vector space
    Eigen::Vector3d psi_init;
    
public:
    GMPCController() : path_available(false) {
        // Initialize subscribers and publisher
        odom_sub = nh.subscribe("/odom", 10, &GMPCController::odomCallback, this);
        path_sub = nh.subscribe("/path_topic", 10, &GMPCController::pathCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // Initialize state and reference
        state = Eigen::Vector3d::Zero();
        ref_state = Eigen::Vector3d::Zero();
        zeta_d = Eigen::Vector3d::Zero();
        
        // Set MPC parameters (tune as needed)
        T = 2.5;
        delta_t = 0.1;
        Q = 10 * Eigen::Matrix3d::Identity();
        Qf = 10 * Eigen::Matrix3d::Identity();
        Rm = 2 * Eigen::Matrix2d::Identity();
        u_min << -0.4, -1.0;
        u_max <<  0.4,  1.0;
        
        // B_const for unicycle at zero orientation: cos0=1, sin0=0.
        B_const << 1.0, 0.0,
                   0.0, 0.0,
                   0.0, 1.0;
    }
    
    // Odometry callback: updates current state and runs MPC if a path is available.
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = tf::getYaw(msg->pose.pose.orientation);
        state << x, y, theta;
        
        if (path_available && !current_path.poses.empty()) {
            updateReferenceState();      // Update ref_state using lookahead
            computeDesiredVelocity();      // Set zeta_d from the reference
            psi_init = state - ref_state;
            psi_init(2) = atan2(sin(psi_init(2)), cos(psi_init(2)));
            
            // Run MPC to compute optimal control input
            Eigen::Vector2d u_opt = runMPC(psi_init, zeta_d);
            
            // Publish control command
            geometry_msgs::Twist cmd;
            cmd.linear.x = u_opt(0);
            cmd.angular.z = u_opt(1);
            cmd_vel_pub.publish(cmd);
        } else {
            // No valid reference trajectory available
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            cmd_vel_pub.publish(cmd);
        }
    }
    
    // Path callback: update the reference trajectory.
    void pathCallback(const nav_msgs::Path::ConstPtr &msg) {
        current_path = *msg;
        if (!current_path.poses.empty()) {
            path_available = true;
        }
    }
    
    // Update reference state: find the closest pose and apply lookahead.
    void updateReferenceState() {
        int closest_idx = 0;
        double min_dist = numeric_limits<double>::max();
        for (size_t i = 0; i < current_path.poses.size(); ++i) {
            double dx = current_path.poses[i].pose.position.x - state(0);
            double dy = current_path.poses[i].pose.position.y - state(1);
            double dist = sqrt(dx*dx + dy*dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        int lookahead = 5;
        int ref_idx = min(closest_idx + lookahead, (int)current_path.poses.size() - 1);
        ref_state(0) = current_path.poses[ref_idx].pose.position.x;
        ref_state(1) = current_path.poses[ref_idx].pose.position.y;
        if (ref_idx > 0) {
            double dx = current_path.poses[ref_idx].pose.position.x - current_path.poses[ref_idx - 1].pose.position.x;
            double dy = current_path.poses[ref_idx].pose.position.y - current_path.poses[ref_idx - 1].pose.position.y;
            ref_state(2) = atan2(dy, dx);
        } else {
            ref_state(2) = 0;
        }
    }
    
    // Compute desired velocity zeta_d based on the reference trajectory.
    void computeDesiredVelocity() {
        int ref_idx = min(0, (int)current_path.poses.size()-1);
        if (ref_idx > 0) {
            double dx = current_path.poses[ref_idx].pose.position.x - current_path.poses[ref_idx-1].pose.position.x;
            double dy = current_path.poses[ref_idx].pose.position.y - current_path.poses[ref_idx-1].pose.position.y;
            double v = sqrt(dx*dx + dy*dy) / delta_t;
            double theta1 = tf::getYaw(current_path.poses[ref_idx].pose.orientation);
            double theta0 = tf::getYaw(current_path.poses[ref_idx-1].pose.orientation);
            double omega = (atan2(sin(theta1 - theta0), cos(theta1 - theta0))) / delta_t;
            zeta_d << v, 0, omega;
        } else {
            zeta_d = Eigen::Vector3d::Zero();
        }
    }
    
    // Compute the adm operator for zeta_d (simplified for a unicycle model)
    Eigen::Matrix3d computeAdm(const Eigen::Vector3d &zeta) {
        Eigen::Matrix3d adm = Eigen::Matrix3d::Zero();
        adm(0,1) = zeta(2);
        adm(1,0) = -zeta(2);
        adm(1,2) = zeta(0);
        return adm;
    }
    
    // Run the MPC optimization using qpOASES.
    Eigen::Vector2d runMPC(const Eigen::Vector3d &psi0, const Eigen::Vector3d &zeta_d) {
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity() + computeAdm(zeta_d) * delta_t;
        Eigen::Matrix<double, 3, 2> B = B_const * delta_t;
        Eigen::Vector3d c = -zeta_d * delta_t;
        
        // Build prediction matrices over the horizon T
        int stateDim = 3;
        int controlDim = 2;
        int nV = T * controlDim;
        int nC = T * stateDim;
        
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nC, stateDim);
        Eigen::MatrixXd M_pred = Eigen::MatrixXd::Zero(nC, nV);
        Eigen::VectorXd d = Eigen::VectorXd::Zero(nC);
        
        Eigen::Matrix3d A_power = Eigen::Matrix3d::Identity();
        for (int k = 0; k < T; k++) {
            A_power = A_power * A;
            S.block(k*stateDim, 0, stateDim, stateDim) = A_power;
            for (int j = 0; j <= k; j++) {
                Eigen::Matrix3d A_temp = Eigen::Matrix3d::Identity();
                for (int m = 0; m < k - j; m++) {
                    A_temp = A_temp * A;
                }
                M_pred.block(k*stateDim, j*controlDim, stateDim, controlDim) = A_temp * B;
            }
            Eigen::Matrix3d A_temp = Eigen::Matrix3d::Identity();
            for (int m = 0; m < k+1; m++) {
                A_temp = A_temp * A;
            }
            d.segment(k*stateDim, stateDim) = A_temp * c;
        }
        
        // Build cost function: J = psi_T' Qf psi_T + sum_{k=0}^{T-1} (psi_k' Q psi_k + u_k' Rm u_k)
        Eigen::MatrixXd H_qp = Eigen::MatrixXd::Zero(nV, nV);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(nV);
        
        for (int k = 0; k < T; k++) {
            Eigen::Matrix3d Qk = (k == T-1) ? Qf : Q;
            Eigen::MatrixXd M_k = M_pred.block(k*stateDim, 0, stateDim, nV);
            Eigen::VectorXd s_k = S.block(k*stateDim, 0, stateDim, stateDim) * psi0 + d.segment(k*stateDim, stateDim);
            H_qp += 2 * (M_k.transpose() * Qk * M_k);
            g += 2 * (M_k.transpose() * Qk * s_k);
        }
        
        Eigen::MatrixXd R_total = Eigen::MatrixXd::Zero(nV, nV);
        for (int k = 0; k < T; k++) {
            R_total.block(k*controlDim, k*controlDim, controlDim, controlDim) = Rm;
        }
        H_qp += 2 * R_total;
        
        Eigen::VectorXd lb = Eigen::VectorXd::Zero(nV);
        Eigen::VectorXd ub = Eigen::VectorXd::Zero(nV);
        for (int k = 0; k < T; k++) {
            lb.segment(k*controlDim, controlDim) = u_min;
            ub.segment(k*controlDim, controlDim) = u_max;
        }
        
        QProblem qp(nV, 0);
        Options options;
        options.printLevel = PL_NONE;
        qp.setOptions(options);
        int nWSR = 100;
        qp.init(H_qp.data(), g.data(), nullptr, lb.data(), ub.data(), nullptr, nullptr, nWSR);
        
        Eigen::VectorXd u_seq = Eigen::VectorXd::Zero(nV);
        qp.getPrimalSolution(u_seq.data());
        
        Eigen::Vector2d u0 = u_seq.segment(0, controlDim);
        return u0;
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
