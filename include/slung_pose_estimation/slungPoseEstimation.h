#ifndef SLUNG_POSE_ESTIMATION_H
#define SLUNG_POSE_ESTIMATION_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include "slung_pose_estimation/State.h"

class SlungPoseEstimation : public rclcpp::Node {
public:
    SlungPoseEstimation();
    ~SlungPoseEstimation();

private:
    // PARAMETERS
    int num_cameras;
    int first_drone_num;
    std::vector<cv::Mat> cam_Ks_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> sub_img_drones_;

    // CALLBACKS
    void clbk_image_common(const sensor_msgs::msg::Image::SharedPtr msg, const int drone_id);
    
    // HELPERS

    // Calculate the camera calibration matrix
    // Inputs: fov_x - horizontal field of view in radians
    //         img_width - image width in pixels
    //         img_height - image height in pixels
    // Outputs: cam_K - 3x3 camera calibration matrix
    void calc_cam_calib_matrix(double fov_x, double img_width, double img_height, cv::Mat &cam_K);
};

#endif // SLUNG_POSE_ESTIMATION_H