#ifndef SLUNG_POSE_MEASUREMENT_H
#define SLUNG_POSE_MEASUREMENT_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include "multi_drone_slung_load_interfaces/msg/phase.hpp"
#include "slung_pose_estimation/State.h"
#include "slung_pose_estimation/utils.h"


class SlungPoseMeasurement : public rclcpp::Node {
public:
    SlungPoseMeasurement();
    ~SlungPoseMeasurement();

private:
    // PARAMETERS
    std::string ns_; // Namespace of the node
    int drone_id_; // ID of the drone this node is running on
    int load_id_;

    int num_drones_;
    int first_drone_num_;

    std::string env_;
    bool evaluate_;           // Whether to evaluate the pose estimation vs ground truth
    int show_markers_config_; // 0 = No, 1 = Yes all, 2 = Drone 1 only
    float marker_edge_length_;
    rclcpp::Time start_time_;

    cv::Mat cam_K_;

    // VARIABLES
    droneState::State state_marker_rel_camera_;
    droneState::State state_expected_pose_measurement_;

    std::string logging_file_path_;

    std::vector<multi_drone_slung_load_interfaces::msg::Phase> drone_phases_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_marker_rel_load_est_;

    
    // Flags 
    bool flag_cam_k_set_ = false;
    bool flag_in_mission_phase_ = false;
    bool flag_expected_pose_measurement_set_ = false;

    // SUBSCRIBERS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_drone_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_color_info_;
    std::vector<rclcpp::Subscription<multi_drone_slung_load_interfaces::msg::Phase>::SharedPtr> sub_phase_drones_;

    // PUBLISHERS
    //rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_marker_rel_camera_;

    // CALLBACKS
    void clbk_cam_color_info_received(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void clbk_image_received(const sensor_msgs::msg::Image::SharedPtr msg);
    void clbk_update_drone_phase(const multi_drone_slung_load_interfaces::msg::Phase::SharedPtr msg, const int drone_index);

    // HELPERS
    void detect_marker(cv::Mat &outputImage, std::vector<cv::Point2f> &targetCorners, const int targetId);
    bool measure_marker_pose(const std::vector<cv::Point2f> &targetCorners, cv::Mat &outputImage);
    void evaluate_pose_measurement();
    void log_pnp_error(const std::string &filename, const droneState::State &state_marker_rel_cam_gt, const droneState::State &state_marker_rel_cam, const droneState::State &state_marker_rel_cam_qs, const droneState::State &state_drone_rel_world, const droneState::State &state_load_rel_world);

    // Calculate the camera calibration matrix
    // Inputs: fov_x - horizontal field of view in radians
    //         img_width - image width in pixels
    //         img_height - image height in pixels
    // Outputs: cam_K - 3x3 camera calibration matrix
    void calc_cam_calib_matrix(double fov_x, double img_width, double img_height, cv::Mat &cam_K);
};

#endif // SLUNG_POSE_MEASUREMENT_H