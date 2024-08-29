#ifndef LOGGER_H
#define LOGGER_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "std_msgs/msg/string.hpp"

#include "slung_pose_estimation/State.h"


class Logger : public rclcpp::Node {
public:
    Logger();
    //~Logger();

private:
    // PARAMETERS
    std::string ns_; // Namespace of the node
    int id_; // ID of the drone this node is running on
    int num_drones_;
    int first_drone_num;

    std::string env_;
    rclcpp::Time start_time_;

    // VARIABLES
    std::vector<droneState::State> states_drones_rel_world_desired;
    std::vector<droneState::State> states_drones_rel_world_gt;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string logging_file_path_;
    
    // SUBSCRIBERS

    // PUBLISHERS

    // CALLBACKS
    void clbk_timer();

    // HELPERS
    void log_gt_poses(const std::string &filename, const droneState::State &state_load_rel_world_gt, const droneState::State &state_load_rel_world_desired, const std::vector<droneState::State> &states_drones_rel_world_gt, const std::vector<droneState::State> &states_drones_rel_world_desired);
};

#endif // LOGGER_H