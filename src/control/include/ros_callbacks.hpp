/**
 * @file callbacks.hpp
 * @author Kartik Sahasrabudhe (kartik.sahasrabudhe1997@gmail.com)
 * @brief Callbacks for ros
 * @version 0.1
 * @date 2025-03-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __ROS_CALLBACKS__
#define __ROS_CALLBACKS__

#include <user_defined_functions.hpp>

// ---------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    float linear_velocity = msg->linear.x;   // Extract linear velocity
    float angular_velocity = msg->angular.z; // Extract angular velocity

    CmdLinearVelocity_mps(linear_velocity, angular_velocity);
    // CmdAngularVelocity_radps(angular_velocity);
}

#endif //__ROS_CALLBACKS__