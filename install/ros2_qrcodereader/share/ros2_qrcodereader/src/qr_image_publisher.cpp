#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class QRImagePublisher : public rclcpp::Node {
public:
    QRImagePublisher() : Node("qr_image_publisher") {

    //default path in case of
    std::string default_image_path = ament_index_cpp::get_package_share_directory("ros2_qrcodereader") + "/imgs/bonjourros2.png";

    this->declare_parameter<std::string>("image_path", default_image_path);
    std::string image_path = this->get_parameter("image_path").as_string();
    RCLCPP_INFO(this->get_logger(), "Image Path : %s", image_path.c_str());

    //try getting image from path
    cv_image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (cv_image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load image at %s", image_path.c_str());
        throw std::runtime_error("Failed to load image");
    }

    //create publisher with 1s fresh rate
    publisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&QRImagePublisher::timer_callback, this));
}

private:
    void timer_callback() {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();
        publisher->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Publishing image");
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    cv::Mat cv_image;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QRImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
