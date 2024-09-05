#ifndef SLUNG_POSE_MEASUREMENT_H
#define SLUNG_POSE_MEASUREMENT_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/string.hpp"

#include "multi_drone_slung_load_interfaces/msg/phase.hpp"
#include "slung_pose_estimation/State.h"
#include "slung_pose_estimation/utils.h"


class SlungPoseEstimationOnline : public rclcpp::Node {
public:
    SlungPoseEstimationOnline();
    //~SlungPoseEstimationOnline();

private:
    // PARAMETERS
    std::string ns_; // Namespace of the node
    int load_id_;

    int num_drones_;
    int first_drone_num_;

    std::string env_;
    rclcpp::Time start_time_;

    // VARIABLES
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_marker_rel_load_est_;

    droneState::State state_current_estimate_;
    
    // std::string logging_file_path_;
    // std::vector<multi_drone_slung_load_interfaces::msg::Phase> drone_phases_;
    
    // Flags 
    //bool flag_in_mission_phase_ = false;

    // SUBSCRIBERS
    //std::vector<rclcpp::Subscription<multi_drone_slung_load_interfaces::msg::Phase>::SharedPtr> sub_phase_drones_;

    // PUBLISHERS
    //rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_marker_rel_camera_;

    // CALLBACKS
    //void clbk_update_drone_phase(const multi_drone_slung_load_interfaces::msg::Phase::SharedPtr msg, const int drone_index);

    // HELPERS
};

#endif // SLUNG_POSE_MEASUREMENT_H