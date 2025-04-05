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

class GMPCController
{
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
    int T;                 // Prediction horizon (number of steps)
    double delta_t;        // Discretization time step
    Eigen::Matrix3d Q;     // State penalty (for intermediate steps)
    Eigen::Matrix3d Qf;    // Terminal state penalty
    Eigen::Matrix2d Rm;    // Control penalty weight
    Eigen::Vector2d u_min; // Lower bound on control input [v, omega]
    Eigen::Vector2d u_max; // Upper bound on control input [v, omega]

    // Desired state reference extracted from the path (lookahead)
    Eigen::Vector3d ref_state;
    // For error computation (state - reference)
    Eigen::Vector3d psi_init;

    // NMPC iterative parameters
    int n_iter; // Maximum number of iterations for successive linearization
    double tol; // Convergence tolerance

public:
    GMPCController() : path_available(false), n_iter(5), tol(1e-3)
    {
        // Initialize subscribers and publisher
        odom_sub = nh.subscribe("/odom", 10, &GMPCController::odomCallback, this);
        path_sub = nh.subscribe("/path_topic", 10, &GMPCController::pathCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Initialize state and reference
        state = Eigen::Vector3d::Zero();
        ref_state = Eigen::Vector3d::Zero();

        // Set MPC parameters (tune as needed)
        T = 10;
        delta_t = 0.1;
        Q = 20 * Eigen::Matrix3d::Identity();
        Qf = 20 * Eigen::Matrix3d::Identity();
        Rm = 2.5 * Eigen::Matrix2d::Identity();
        u_min << -0.4, -1.0;
        u_max << 0.4, 1.0;
    }

    // Odometry callback: updates current state and runs MPC if a path is available.
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = tf::getYaw(msg->pose.pose.orientation);
        state << x, y, theta;

        if (path_available && !current_path.poses.empty())
        {
            updateReferenceState(); // Update ref_state using lookahead
            psi_init = state - ref_state;
            psi_init(2) = atan2(sin(psi_init(2)), cos(psi_init(2)));

            // Run NMPC to compute optimal control input
            Eigen::Vector2d u_opt = runNMPC(state, psi_init);

            // Publish control command
            geometry_msgs::Twist cmd;
            cmd.linear.x = u_opt(0);
            cmd.angular.z = u_opt(1);
            cmd_vel_pub.publish(cmd);
        }
        else
        {
            // No valid reference trajectory available: command zero velocities.
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            cmd_vel_pub.publish(cmd);
        }
    }

    // Path callback: update the reference trajectory.
    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        current_path = *msg;
        if (!current_path.poses.empty())
        {
            path_available = true;
        }
    }

    // Update reference state: find the closest pose and apply lookahead.
    void updateReferenceState()
    {
        int closest_idx = 0;
        double min_dist = numeric_limits<double>::max();
        for (size_t i = 0; i < current_path.poses.size(); ++i)
        {
            double dx = current_path.poses[i].pose.position.x - state(0);
            double dy = current_path.poses[i].pose.position.y - state(1);
            double dist = sqrt(dx * dx + dy * dy);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }
        int lookahead = 1;
        int ref_idx = min(closest_idx + lookahead, (int)current_path.poses.size() - 1);
        ref_state(0) = current_path.poses[ref_idx].pose.position.x;
        ref_state(1) = current_path.poses[ref_idx].pose.position.y;
        if (ref_idx > 0)
        {
            double dx = current_path.poses[ref_idx].pose.position.x - current_path.poses[ref_idx - 1].pose.position.x;
            double dy = current_path.poses[ref_idx].pose.position.y - current_path.poses[ref_idx - 1].pose.position.y;
            ref_state(2) = atan2(dy, dx);
        }
        else
        {
            ref_state(2) = 0;
        }
    }

    // The nonlinear dynamics of the unicycle.
    // state = [x, y, theta] and u = [v, omega]
    Eigen::Vector3d f(const Eigen::Vector3d &state, const Eigen::Vector2d &u)
    {
        Eigen::Vector3d next;
        double theta = state(2);
        next(0) = state(0) + delta_t * u(0) * cos(theta);
        next(1) = state(1) + delta_t * u(0) * sin(theta);
        next(2) = state(2) + delta_t * u(1);
        return next;
    }

    // Compute the Jacobian matrices A and B at a given state and input.
    // A = df/dstate, B = df/du.
    void computeJacobians(const Eigen::Vector3d &state, const Eigen::Vector2d &u,
                          Eigen::Matrix3d &A, Eigen::Matrix<double, 3, 2> &B)
    {
        double theta = state(2);
        double v = u(0);
        A = Eigen::Matrix3d::Identity();
        A(0, 2) = -delta_t * v * sin(theta);
        A(1, 2) = delta_t * v * cos(theta);

        B = Eigen::Matrix<double, 3, 2>::Zero();
        B(0, 0) = delta_t * cos(theta);
        B(1, 0) = delta_t * sin(theta);
        B(2, 1) = delta_t;
    }

    // Run the NMPC using successive linearization.
    // We linearize about a predicted trajectory (initialized with zeros) and solve a QP.
    Eigen::Vector2d runNMPC(const Eigen::Vector3d &init_state, const Eigen::Vector3d &error_state)
    {
        // Dimensions:
        int stateDim = 3;
        int controlDim = 2;
        int nV = T * controlDim; // Total control inputs over horizon
        int nC = T * stateDim;   // Total state constraints (for cost prediction)

        // Initial guess for control sequence (zero input)
        Eigen::VectorXd u_seq = Eigen::VectorXd::Zero(nV);

        // Iterative linearization loop
        for (int iter = 0; iter < n_iter; iter++)
        {
            // Predict state trajectory using current control sequence (nonlinear propagation)
            vector<Eigen::Vector3d> x_pred(T + 1);
            x_pred[0] = init_state;
            for (int k = 0; k < T; k++)
            {
                Eigen::Vector2d u_k = u_seq.segment(k * controlDim, controlDim);
                x_pred[k + 1] = f(x_pred[k], u_k);
            }

            // Build the time-varying prediction matrices.
            Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nC, stateDim); // free response
            Eigen::MatrixXd M_pred = Eigen::MatrixXd::Zero(nC, nV);  // control influence

            // We will also build an offset term from the nonlinear dynamics.
            // Let δx_k = x_k - x̄_k, where x̄_k is the predicted state from previous iteration.
            // The linearized dynamics: δx_{k+1} ≈ A_k δx_k + B_k δu_k + d_k, where d_k accounts for linearization error.
            // Here, we set d_k = x_pred[k+1] - (A_k x_pred[k] + B_k u_k) so that at the predicted trajectory, the error is zero.
            Eigen::VectorXd d = Eigen::VectorXd::Zero(nC);

            // For the first step, the deviation is psi0.
            // We accumulate the effect of the dynamics along the horizon.
            Eigen::MatrixXd A_bar = Eigen::MatrixXd::Identity(stateDim, stateDim);

            // We'll store A_k and B_k for each step.
            vector<Eigen::Matrix3d> A_list;
            vector<Eigen::Matrix<double, 3, 2>> B_list;
            A_list.resize(T);
            B_list.resize(T);

            for (int k = 0; k < T; k++)
            {
                Eigen::Matrix3d A_k;
                Eigen::Matrix<double, 3, 2> B_k;
                // Linearize at predicted state x_pred[k] and control u_seq[k]
                computeJacobians(x_pred[k], u_seq.segment(k * controlDim, controlDim), A_k, B_k);
                A_list[k] = A_k;
                B_list[k] = B_k;
            }

            // Now build S and M_pred recursively.
            for (int k = 0; k < T; k++)
            {
                if (k == 0)
                {
                    S.block(0, 0, stateDim, stateDim) = A_list[0];
                    M_pred.block(0, 0, stateDim, controlDim) = B_list[0];
                    // Offset: d_0 = x_pred[1] - (A_0*x_pred[0] + B_0*u_0)
                    d.segment(0, stateDim) = x_pred[1] - (A_list[0] * x_pred[0] + B_list[0] * u_seq.segment(0, controlDim));
                    A_bar = A_list[0];
                }
                else
                {
                    S.block(k * stateDim, 0, stateDim, stateDim) = A_bar * A_list[k];
                    // Sum over j=0 to k: M_pred[k] = A_{k-1}...A_{j+1} B_j
                    for (int j = 0; j <= k; j++)
                    {
                        Eigen::MatrixXd prod = Eigen::MatrixXd::Identity(stateDim, stateDim);
                        for (int m = j + 1; m <= k; m++)
                        {
                            prod = prod * A_list[m];
                        }
                        M_pred.block(k * stateDim, j * controlDim, stateDim, controlDim) = prod * B_list[j];
                    }
                    // Compute offset d for this step:
                    // d_k = x_pred[k+1] - (A_bar*A_list[k] * x_pred[k] + sum_{j=0}^{k} prod_{m=j+1}^{k} A_list[m] B_list[j] u_seq_j)
                    // We already have x_pred computed, so we can set:
                    d.segment(k * stateDim, stateDim) = x_pred[k + 1] - (S.block(k * stateDim, 0, stateDim, stateDim) * x_pred[0] + M_pred.block(k * stateDim, 0, stateDim, nV) * u_seq);
                    A_bar = S.block(k * stateDim, 0, stateDim, stateDim);
                }
            }

            // Build the cost function:
            // J = (x_T - x_ref)' Qf (x_T - x_ref) + sum_{k=0}^{T-1} [ (x_k - x_ref)' Q (x_k - x_ref) + u_k' Rm u_k ]
            // Here we work in the error coordinates δx. The predicted error at step k is:
            // δx_k = S_k * (x0 - x_pred[0]) + M_pred_k * δu + d_k, but note that we have x_pred as our linearization point.
            // For simplicity, let psi0 = (init_state - ref_state). Then we set up the QP in δu.

            Eigen::MatrixXd H_qp = Eigen::MatrixXd::Zero(nV, nV);
            Eigen::VectorXd g = Eigen::VectorXd::Zero(nV);

            for (int k = 0; k < T; k++)
            {
                Eigen::Matrix3d Qk = (k == T - 1) ? Qf : Q;
                Eigen::MatrixXd M_k = M_pred.block(k * stateDim, 0, stateDim, nV);
                // s_k = S_k * psi0 + d_k. Here we assume psi0 = (init_state - ref_state).
                Eigen::VectorXd s_k = S.block(k * stateDim, 0, stateDim, stateDim) * psi_init + d.segment(k * stateDim, stateDim);
                H_qp += 2 * (M_k.transpose() * Qk * M_k);
                g += 2 * (M_k.transpose() * Qk * s_k);
            }

            // Add control cost term.
            Eigen::MatrixXd R_total = Eigen::MatrixXd::Zero(nV, nV);
            for (int k = 0; k < T; k++)
            {
                R_total.block(k * controlDim, k * controlDim, controlDim, controlDim) = Rm;
            }
            H_qp += 2 * R_total;

            // Set bounds on control increments.
            Eigen::VectorXd lb = Eigen::VectorXd::Zero(nV);
            Eigen::VectorXd ub = Eigen::VectorXd::Zero(nV);
            for (int k = 0; k < T; k++)
            {
                lb.segment(k * controlDim, controlDim) = u_min;
                ub.segment(k * controlDim, controlDim) = u_max;
            }

            // Solve the QP using qpOASES.
            QProblem qp(nV, 0);
            Options options;
            options.printLevel = PL_NONE;
            qp.setOptions(options);
            int nWSR = 100;
            qp.init(H_qp.data(), g.data(), nullptr, lb.data(), ub.data(), nullptr, nullptr, nWSR);

            Eigen::VectorXd u_seq_new = Eigen::VectorXd::Zero(nV);
            qp.getPrimalSolution(u_seq_new.data());

            // Check for convergence (difference in control sequence)
            if ((u_seq_new - u_seq).norm() < tol)
            {
                u_seq = u_seq_new;
                break;
            }
            u_seq = u_seq_new;
        }

        // Return the first control input from the optimized sequence.
        Eigen::Vector2d u0 = u_seq.segment(0, controlDim);
        return u0;
    }

    void run()
    {
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gmpc_controller");
    GMPCController controller;
    controller.run();
    return 0;
}