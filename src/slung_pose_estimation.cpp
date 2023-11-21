#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("image_subscriber") {
        // Get parameters
        this->declare_parameter<int>("num_cameras", 3); // Default to 3
        this->get_parameter("num_cameras", this->num_cameras);

        this->declare_parameter<int>("first_drone_num", 1); // Default to 1
        this->get_parameter("first_drone_num", this->first_drone_num);

        // Subscribe to image topics
        for (int i = first_drone_num; i < num_cameras+1; i++) {
            std::string topic_name = "/x500_" + std::to_string(i) + "/camera";

            // Store the subsciptions in a vector
            sub_img_drones_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
                topic_name, 10, [this, i](const sensor_msgs::msg::Image::SharedPtr msg)
                { this->image_callback_common(msg, i); }));   
        }

        // Get the camera calibration matrix for each drone/camera (TODO: Replace with subscription to camera_info topic)
        for (int i = first_drone_num; i < num_cameras + 1; i++) {
            cv::Mat cam_K = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0,
                                            0.0, 0.0, 1.0);
            calc_cam_calib_matrix(1.396, 960, 540, cam_K);
            cam_Ks_.push_back(cam_K);

            // Print the matrix
            //std::cout << "Camera " << i << " Calibration Matrix:\n" << cam_K << "\n\n";
            // Convert cv::Mat to string for logging

            // std::stringstream ss;
            // ss << cam_K;
            // std::string matAsString = ss.str();

            // // Log the matrix using ROS 2 logging
            // RCLCPP_INFO(this->get_logger(), "Cal matrix %d: \n%s", i, matAsString.c_str());

        }

        // Create a window to display the image
        cv::namedWindow("Detected Markers", cv::WINDOW_AUTOSIZE);
    }

    ~ImageSubscriber() {
        // Destroy the window when the object is destroyed
        cv::destroyWindow("Drone 1 Marker Detection");
    }

private:
    /*
    * PARAMETERS
    */
    int num_cameras;
    int first_drone_num;

    // Camera intrinsic calibration matrices
    std::vector<cv::Mat> cam_Ks_;
    

    /*
    * SUBSCRIBERS
    */
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> sub_img_drones_;


    /*
    * CALLBACKS
    */
    // Callback function for image subscriber
    void image_callback_common(const sensor_msgs::msg::Image::SharedPtr msg, const int drone_id) {
        RCLCPP_INFO(this->get_logger(), "Received image from drone %d", drone_id);
        
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

        // Print detected marker IDs
        // std::stringstream ss;
        // for (const auto &id : markerIds) {
        //     ss << id << " ";
        // }
        // RCLCPP_INFO(this->get_logger(), "Detected marker IDs: %s", ss.str().c_str());


        // Perform marker pose estimation if the target marker is detected
        if (!targetCorners.empty()) {
            // Define the 3D coordinates of marker corners (assuming square markers) in the marker's coordinate system
            float markerEdgeLength = 0.04f;  // marker size in meters

            std::vector<cv::Point3f> markerPoints = {
                {-markerEdgeLength/2,  markerEdgeLength/2, 0.0f},
                { markerEdgeLength/2,  markerEdgeLength/2, 0.0f},
                { markerEdgeLength/2,  -markerEdgeLength/2, 0.0f},
                {-markerEdgeLength/2, -markerEdgeLength/2, 0.0f}
            };

            // Get the camera parameters 
            cv::Mat cam_K = cam_Ks_[drone_id - 1];
            cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F); // Assuming no distortion

            // Using PnP, estimate pose T^c_m (marker pose relative to camera) for the target marker
            cv::Vec3d rvec, tvec;
            cv::solvePnP(markerPoints, targetCorners, cam_K, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE); // cv::SOLVEPNP_ITERATIVE // Could alternatively use old cv::aruco::estimatePoseSingleMarkers (note this defaults to using ITERATIVE: https://github.com/opencv/opencv_contrib/blob/4.x/modules/aruco/include/opencv2/aruco.hpp)

            // Display the image (for drone 1) with detected markers
            if (drone_id == 1){
                // Draw the detected marker axes
                cv::drawFrameAxes(outputImage, cam_K, distCoeffs, rvec, tvec, 0.1);

                cv::imshow("Detected Markers", outputImage);
                cv::waitKey(30);
            }
        }
    }


    /*
    * HELPERS
    */
    // Calculate the camera calibration matrix
    // Inputs: fov_x - horizontal field of view in radians
    //         img_width - image width in pixels
    //         img_height - image height in pixels
    // Outputs: cam_K - 3x3 camera calibration matrix
    void calc_cam_calib_matrix(double fov_x, double img_width, double img_height, cv::Mat &cam_K) {
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

    
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}