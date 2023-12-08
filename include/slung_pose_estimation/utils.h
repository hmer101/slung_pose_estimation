#ifndef UTILS_H
#define UTILS_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <string>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/msg/string.hpp"

#include "slung_pose_estimation/State.h"


namespace utils {
    // TRANSFORMS
    std::optional<geometry_msgs::msg::TransformStamped> lookup_tf(const std::string &target_frame, const std::string &source_frame, tf2_ros::Buffer &tfBuffer, const rclcpp::Time& time, rclcpp::Logger logger);

    Eigen::Vector3d transform_position(const Eigen::Vector3d& p_BA, const Eigen::Vector3d& p_CB, const tf2::Quaternion& q_CB);
    tf2::Quaternion transform_orientation(const tf2::Quaternion& q_BA, const tf2::Quaternion& q_CB);
    //std::shared_ptr<State> transform_frames(const State& state, const std::string& frame2_name, tf2_ros::Buffer& tf_buffer, rclcpp::Logger logger);
    std::shared_ptr<droneState::State> transform_frames(const droneState::State &state, const std::string &frame2_name, tf2_ros::Buffer &tf_buffer, rclcpp::Logger logger, droneState::CS_type cs_out_type = droneState::CS_type::XYZ);

    // MATH
    float getTrace(const tf2::Matrix3x3 &matrix);

    // STRING HANDLING
    int extract_id_from_name(const std::string &input);
    std::vector<float> splitAndConvert(const std::string &s, char delimiter);

    // CONVERSIONS
    geometry_msgs::msg::Pose convert_state_to_pose_msg(const droneState::State &state);
    tf2::Quaternion convert_rvec_to_quaternion(const cv::Vec3d &rvec);
}

#endif // UTILS_H
