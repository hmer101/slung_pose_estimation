#include "slung_pose_estimation/slungPoseMeasurement.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>


SlungPoseMeasurement::SlungPoseMeasurement() : Node("slung_pose_measure", rclcpp::NodeOptions().use_global_arguments(true)) {
    // PARAMETERS
    this->ns_ = this->get_namespace();
    this->drone_id_ = utils::extract_id_from_name(this->ns_);

    this->declare_parameter<int>("show_markers", 0);
    this->get_parameter("show_markers", this->show_markers_config_); 

    this->declare_parameter<int>("load_id", 1);
    this->get_parameter("load_id", this->load_id_);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("slung_pose_estimation");
    this->logging_file_path_ = package_share_directory + "/data/pnp_errors.txt";

    // VARIABLES
    this->state_marker_rel_camera_ = droneState::State("camera" + std::to_string(this->drone_id_), droneState::CS_type::XYZ);

    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); //tf2_ros::Buffer(std::make_shared<rclcpp::Clock>());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer_));


    // SUBSCRIBERS
    std::string topic_name = this->ns_ + "/out/camera";

    this->sub_img_drone_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 10, std::bind(&SlungPoseMeasurement::clbk_image_received, this, std::placeholders::_1));


    // PUBLISHERS
    this->pub_marker_rel_camera_ = this->create_publisher<geometry_msgs::msg::Pose>(
        this->ns_ + "/out/marker_rel_camera", 10);


    // SETUP
    this->start_time_ = this->get_clock()->now();

    // Get the camera calibration matrix for the camera (TODO: Replace with subscription to camera_info topic)
    this->cam_K_ = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0);
    calc_cam_calib_matrix(1.396, 960, 540, this->cam_K_);

    // Create a window to display the image if desired
    if (this->show_markers_config_ == 1 || (this->show_markers_config_ == 2 && this->drone_id_ == 1)){
        cv::namedWindow("Detected Markers Drone " + std::to_string(this->drone_id_), cv::WINDOW_AUTOSIZE);
    }

    // Print info
    RCLCPP_INFO(this->get_logger(), "MEASUREMENT NODE %d", this->drone_id_);
    
}


SlungPoseMeasurement::~SlungPoseMeasurement() {
    // Destroy the window when the object is destroyed
    cv::destroyWindow("Detected Markers Drone " + std::to_string(this->drone_id_));
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
        // TODO: Properly parametize in a file to allow easy swapping to real world
        float markerEdgeLength = 0.08f;  // marker size in meters

        std::vector<cv::Point3f> markerPoints = {
            {-markerEdgeLength/2,  markerEdgeLength/2, 0.0f},
            { markerEdgeLength/2,  markerEdgeLength/2, 0.0f},
            { markerEdgeLength/2,  -markerEdgeLength/2, 0.0f},
            {-markerEdgeLength/2, -markerEdgeLength/2, 0.0f}
        };

        // Get the camera parameters 
        cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // Assuming no distortion

        // Using PnP, estimate pose T^c_m (marker pose relative to camera) for the target marker
        // Could alternatively use old cv::aruco::estimatePoseSingleMarkers (note this defaults to using ITERATIVE: https://github.com/opencv/opencv_contrib/blob/4.x/modules/aruco/include/opencv2/aruco.hpp)
        cv::Vec3d rvec, tvec;
        cv::solvePnP(markerPoints, targetCorners, this->cam_K_, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE); //cv::SOLVEPNP_P3P cv::SOLVEPNP_IPPE_SQUARE // cv::SOLVEPNP_ITERATIVE 

        // Display the image (for drone 1) with detected markers
        if (this->show_markers_config_ == 1 || (this->show_markers_config_ == 2 && this->drone_id_ == 1)){
            // Draw the detected marker axes
            cv::drawFrameAxes(outputImage, this->cam_K_, distCoeffs, rvec, tvec, 0.1);

            cv::imshow("Detected Markers Drone " + std::to_string(this->drone_id_), outputImage);
            cv::waitKey(30);
        }

        this->state_marker_rel_camera_.setAtt(utils::convert_rvec_to_quaternion(rvec));
        this->state_marker_rel_camera_.setPos(Eigen::Vector3d(tvec[0], tvec[1], tvec[2]));

        // Publish measured marker pose rel camera
        geometry_msgs::msg::Pose pose_msg = utils::convert_state_to_pose_msg(this->state_marker_rel_camera_);
        this->pub_marker_rel_camera_->publish(pose_msg);

        // Evaluate the marker pose estimation against ground truth
        auto marker_gt_rel_cam_gt = utils::lookup_tf("camera" + std::to_string(this->drone_id_) + "_gt","load_marker" + std::to_string(this->load_id_) + "_gt", *this->tf_buffer_, rclcpp::Time(0), this->get_logger());

        if (marker_gt_rel_cam_gt && (this->show_markers_config_ == 1 || (this->show_markers_config_ == 2 && this->drone_id_ == 1))) {
            // Calculate the PnP error            
            auto state_marker_rel_cam_gt = droneState::State("camera" + std::to_string(this->drone_id_) + "_gt", droneState::CS_type::XYZ);
            state_marker_rel_cam_gt.setPos(Eigen::Vector3d(marker_gt_rel_cam_gt->transform.translation.x, marker_gt_rel_cam_gt->transform.translation.y, marker_gt_rel_cam_gt->transform.translation.z));
            state_marker_rel_cam_gt.setAtt(tf2::Quaternion(marker_gt_rel_cam_gt->transform.rotation.x, marker_gt_rel_cam_gt->transform.rotation.y, marker_gt_rel_cam_gt->transform.rotation.z, marker_gt_rel_cam_gt->transform.rotation.w));

            // Save the PnP error data to a file
            this->log_pnp_error(this->logging_file_path_, state_marker_rel_cam_gt, this->state_marker_rel_camera_);

            // PRINTING FOR DEBUGGING
            // Print ground truth
            // double yaw_gt, pitch_gt, roll_gt;
            // state_marker_rel_cam_gt.getAttYPR(yaw_gt, pitch_gt, roll_gt);

            // yaw_gt = yaw_gt * 180.0 / M_PI;
            // pitch_gt = pitch_gt * 180.0 / M_PI;
            // roll_gt = roll_gt * 180.0 / M_PI;

            // RCLCPP_INFO(this->get_logger(), "Marker pose rel cam ground truth: %f %f %f %f %f %f",
            //             state_marker_rel_cam_gt.getPos()[0], state_marker_rel_cam_gt.getPos()[1], state_marker_rel_cam_gt.getPos()[2],
            //             roll_gt, pitch_gt, yaw_gt);

            // Print the measured pose
            // double yaw_meas, pitch_meas, roll_meas;
            // this->state_marker_rel_camera_.getAttYPR(yaw_meas, pitch_meas, roll_meas);

            // yaw_meas = yaw_meas * 180.0 / M_PI;
            // pitch_meas = pitch_meas * 180.0 / M_PI;
            // roll_meas = roll_meas * 180.0 / M_PI;

            // RCLCPP_INFO(this->get_logger(), "Marker pose rel cam measured: %f %f %f %f %f %f",
            //             this->state_marker_rel_camera_.getPos()[0], this->state_marker_rel_camera_.getPos()[1], this->state_marker_rel_camera_.getPos()[2],
            //             roll_meas, pitch_meas, yaw_meas);

        }
    }    
}

void SlungPoseMeasurement::log_pnp_error(const std::string &filename, const droneState::State& state_marker_rel_cam_gt, const droneState::State& state_marker_rel_cam){ //const std::string& filename, double distTrans, double distAngGeo) {
    std::ofstream logFile;
    logFile.open(filename, std::ios::app); // Open in append mode

    // Get data to log
    float time = this->get_clock()->now().seconds() - this->start_time_.seconds();

    Eigen::Vector3d pos_gt = state_marker_rel_cam_gt.getPos();
    Eigen::Vector3d pos = state_marker_rel_cam.getPos();

    Eigen::Vector3d rpy_gt = state_marker_rel_cam_gt.getAttYPR();
    Eigen::Vector3d rpy = state_marker_rel_cam.getAttYPR();
    
    Eigen::Vector3d pos_err = pos - pos_gt;
    Eigen::Vector3d att_err = rpy - rpy_gt;

    float distTrans = state_marker_rel_cam.distTrans(state_marker_rel_cam_gt);
    float distAngGeo = state_marker_rel_cam.distAngGeo(state_marker_rel_cam_gt)*180.0 / M_PI;
    
    if (logFile.is_open()) {
        logFile << time << " " 
                << pos_gt.x() << " " << pos_gt.y() << " " << pos_gt.z() << " "
                << rpy_gt.x() << " " << rpy_gt.y() << " " << rpy_gt.z() << " "
                << pos.x() << " " << pos.y() << " " << pos.z() << " "
                << rpy.x() << " " << rpy.y() << " " << rpy.z() << " "
                << pos_err.x() << " " << pos_err.y() << " " << pos_err.z() << " "
                << att_err.x() << " " << att_err.y() << " " << att_err.z() << " "
                << distTrans << " " << distAngGeo << std::endl;
        logFile.close();
    }else {
        RCLCPP_ERROR(this->get_logger(), "Unable to open file for logging");
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