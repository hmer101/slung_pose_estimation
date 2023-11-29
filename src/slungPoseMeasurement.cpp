#include "slung_pose_estimation/slungPoseMeasurement.h"

SlungPoseMeasurement::SlungPoseMeasurement() : Node("slung_pose_measure", rclcpp::NodeOptions().use_global_arguments(true)) {
    //PARAMETERS
    this->ns_ = this->get_namespace();
    this->drone_id_ = utils::extract_id_from_name(this->ns_);

    this->declare_parameter<int>("show_markers", 0);
    this->get_parameter("show_markers", this->show_markers_config_); 

    //VARIABLES
    this->state_marker_rel_camera_ = State("camera" + this->drone_id_, CS_type::XYZ);

    // SUBSCRIBERS
    std::string topic_name = this->ns_ + "/out/camera";

    this->sub_img_drone_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 10, std::bind(&SlungPoseMeasurement::clbk_image_received, this, std::placeholders::_1));


    // PUBLISHERS
    this->pub_marker_rel_camera_ = this->create_publisher<geometry_msgs::msg::Pose>(
        this->ns_ + "/out/marker_rel_camera", 10);


    // SETUP
    // Get the camera calibration matrix for the camera (TODO: Replace with subscription to camera_info topic)
    this->cam_K_ = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0);
    calc_cam_calib_matrix(1.396, 960, 540, this->cam_K_);

    // Create a window to display the image if desired
    if (this->show_markers_config_ == 1 || (this->show_markers_config_ == 2 && this->drone_id_ == 1)){
        cv::namedWindow("Detected Markers Drone " + this->drone_id_, cv::WINDOW_AUTOSIZE);
    }

    // Print info
    RCLCPP_INFO(this->get_logger(), "MEASUREMENT NODE %d", this->drone_id_);
    //RCLCPP_INFO(this->get_logger(), "Show markers config %d", this->show_markers_config_);
    
}


SlungPoseMeasurement::~SlungPoseMeasurement() {
    // Destroy the window when the object is destroyed
    cv::destroyWindow("Detected Markers Drone " + this->drone_id_);
}

void SlungPoseMeasurement::clbk_image_received(const sensor_msgs::msg::Image::SharedPtr msg){
    // Convert ROS Image message to OpenCV image
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // Detect ArUco markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

    cv::Mat outputImage = image.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    // Extract the corners of the target marker
    int targetId = 1; // Marker ID 1 is on top of the load
    std::vector<cv::Point2f> targetCorners; 

    // Find the index of the target ID and extract corresponding corners
    for (size_t i = 0; i < markerIds.size(); i++) {
        if (markerIds[i] == targetId) {
            targetCorners = markerCorners[i];
            break; // Assuming only one set of corners per marker ID
        }
    }

    // Perform marker pose estimation if the target marker is detected
    if (!targetCorners.empty()) {
        // Define the 3D coordinates of marker corners (assuming square markers) in the marker's coordinate system
        float markerEdgeLength = 0.08f;  // marker size in meters

        std::vector<cv::Point3f> markerPoints = {
            {-markerEdgeLength/2,  markerEdgeLength/2, 0.0f},
            { markerEdgeLength/2,  markerEdgeLength/2, 0.0f},
            { markerEdgeLength/2,  -markerEdgeLength/2, 0.0f},
            {-markerEdgeLength/2, -markerEdgeLength/2, 0.0f}
        };

        // Get the camera parameters 
        //cv::Mat cam_K = cam_Ks_[drone_id - 1];
        cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // Assuming no distortion

        // Using PnP, estimate pose T^c_m (marker pose relative to camera) for the target marker
        cv::Vec3d rvec, tvec;
        cv::solvePnP(markerPoints, targetCorners, this->cam_K_, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE); // cv::SOLVEPNP_ITERATIVE // Could alternatively use old cv::aruco::estimatePoseSingleMarkers (note this defaults to using ITERATIVE: https://github.com/opencv/opencv_contrib/blob/4.x/modules/aruco/include/opencv2/aruco.hpp)

        // Display the image (for drone 1) with detected markers
        if (this->show_markers_config_ == 1 || (this->show_markers_config_ == 2 && this->drone_id_ == 1)){
            // Draw the detected marker axes
            cv::drawFrameAxes(outputImage, this->cam_K_, distCoeffs, rvec, tvec, 0.1);

            cv::imshow("Detected Markers Drone " + std::to_string(this->drone_id_), outputImage);
            cv::waitKey(30);
        }

        // Convert marker pose to State object
        this->state_marker_rel_camera_.setAtt(utils::convert_rvec_to_quaternion(rvec));
        this->state_marker_rel_camera_.setPos(Eigen::Vector3d(tvec[0], tvec[1], tvec[2]));

        // Publish measured marker pose rel camera
        geometry_msgs::msg::Pose pose_msg = utils::convert_state_to_pose_msg(this->state_marker_rel_camera_);
        this->pub_marker_rel_camera_->publish(pose_msg);

        //RCLCPP_INFO(this->get_logger(), "Published marker pose rel drone %d", this->drone_id_);
    }

    
}

void SlungPoseMeasurement::calc_cam_calib_matrix(double fov_x, double img_width, double img_height, cv::Mat &cam_K) {
    // Calculate the aspect ratio
    double aspect_ratio = img_width / img_height;

    // Calculate the focal length in pixels
    double focal_length_x = img_width / (2.0 * tan(fov_x / 2.0));
    double focal_length_y = focal_length_x / aspect_ratio;

    // Optical center coordinates
    double c_x = img_width / 2.0;
    double c_y = img_height / 2.0;

    // Ensure cam_K is of the correct size and type
    cam_K.create(3, 3, CV_64F);

    // Fill out the calibration matrix
    cam_K = (cv::Mat_<double>(3, 3) << focal_length_x, 0.0, c_x,
                                        0.0, focal_length_y, c_y,
                                        0.0, 0.0, 1.0);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlungPoseMeasurement>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}