#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("image_subscriber") {
        subscription_1_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/x500_1/camera", 10,
            std::bind(&ImageSubscriber::image_callback_1, this, std::placeholders::_1));

        subscription_2_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/x500_2/camera", 10,
            std::bind(&ImageSubscriber::image_callback_2, this, std::placeholders::_1));

        subscription_3_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/x500_3/camera", 10,
            std::bind(&ImageSubscriber::image_callback_3, this, std::placeholders::_1));
    }

private:
    void image_callback_1(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received image 1");
    }

    void image_callback_2(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received image 2");
    }

    void image_callback_3(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received image 3");
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_2_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_3_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}