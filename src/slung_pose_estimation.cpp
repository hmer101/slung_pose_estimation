#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("image_subscriber") {
        // Get parameters
        this->declare_parameter<int>("num_cameras", 3); // Default to 3
        int num_cameras;
        this->get_parameter("num_cameras", num_cameras);

        this->declare_parameter<int>("first_drone_num", 1); // Default to 1
        int first_drone_num;
        this->get_parameter("first_drone_num", first_drone_num);

        // Subscribe to image topics
        for (int i = first_drone_num; i < num_cameras+1; i++) {
            std::string topic_name = "/x500_" + std::to_string(i) + "/camera";

            // Store the subsciptions in a vector
            sub_img_drones_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
                topic_name, 10, [this, i](const sensor_msgs::msg::Image::SharedPtr msg)
                { this->image_callback_common(msg, i); }));   
        }

        // Create a window to display the image
        cv::namedWindow("Detected Markers", cv::WINDOW_AUTOSIZE);
    }

    ~ImageSubscriber() {
        // Destroy the window when the object is destroyed
        cv::destroyWindow("Drone 1 Marker Detection");
    }

private:
    void image_callback_common(const sensor_msgs::msg::Image::SharedPtr msg, const int drone_id) {
        RCLCPP_INFO(this->get_logger(), "Received image from drone %d", drone_id);
        
        // Convert ROS Image message to OpenCV image
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Detect ArUco marker
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

        cv::Mat outputImage = image.clone();
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

        // if (!markerIds.empty()) {
        //     // Assuming camera parameters (cameraMatrix, distCoeffs) are known
        //     std::vector<cv::Vec3d> rvecs, tvecs;
        //     cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);

        //     // Use tvecs for the position of the marker relative to the camera
        //     // Transform this position to the drone's coordinate system if needed
        // }

        // Display the image (for drone 1) with detected markers
        if (drone_id == 1){
            // Print detected marker IDs
            std::stringstream ss;
            for (const auto &id : markerIds) {
                ss << id << " ";
            }
            RCLCPP_INFO(this->get_logger(), "Detected marker IDs: %s", ss.str().c_str());

            cv::imshow("Detected Markers", outputImage);
            cv::waitKey(30);
        }
   
    }

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> sub_img_drones_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}