#ifndef SLUNG_POSE_MEASUREMENT_H
#define SLUNG_POSE_MEASUREMENT_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include "slung_pose_estimation/State.h"
#include "slung_pose_estimation/utils.h"


class SlungPoseMeasurement : public rclcpp::Node {
public:
    SlungPoseMeasurement();
    ~SlungPoseMeasurement();

private:
    // PARAMETERS
    //int num_cameras;
    //int first_drone_num;
    //std::vector<cv::Mat> cam_Ks_;
    //std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> sub_img_drones_;

    std::string ns_; // Namespace of the node
    int drone_id_; // ID of the drone this node is running on
    int show_markers_config_; // 0 = No, 1 = Yes all, 2 = Drone 1 only

    cv::Mat cam_K_;

    // VARIABLES
    State state_marker_rel_camera_;

    // SUBSCRIBERS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_drone_;

    // PUBLISHERS
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_marker_rel_camera_;

    // CALLBACKS
    void clbk_image_received(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // HELPERS

    // Calculate the camera calibration matrix
    // Inputs: fov_x - horizontal field of view in radians
    //         img_width - image width in pixels
    //         img_height - image height in pixels
    // Outputs: cam_K - 3x3 camera calibration matrix
    void calc_cam_calib_matrix(double fov_x, double img_width, double img_height, cv::Mat &cam_K);
};

#endif // SLUNG_POSE_MEASUREMENT_H