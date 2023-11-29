#ifndef UTILS_H
#define UTILS_H

#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/string.hpp"

#include "slung_pose_estimation/State.h"

// Forward declaration
//class State;

namespace utils {
    // TRANSFORMS
    Eigen::Vector3d transform_position(const Eigen::Vector3d& p_BA, const Eigen::Vector3d& p_CB, const tf2::Quaternion& q_CB);
    tf2::Quaternion transform_orientation(const tf2::Quaternion& q_BA, const tf2::Quaternion& q_CB);
    std::shared_ptr<State> transform_frames(const State& state, const std::string& frame2_name, tf2_ros::Buffer& tf_buffer, rclcpp::Logger logger);

    // STRING HANDLING
    int extract_id_from_name(const std::string &input);

    // CONVERSIONS
    geometry_msgs::msg::Pose convert_state_to_pose_msg(const State &state);
}

#endif // UTILS_H
