#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

class WebcamPublisher : public rclcpp::Node {
public:
    WebcamPublisher() : Node("webcam_publisher"), capture(30) {

        if (!capture.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the webcam");
            rclcpp::shutdown();
        }

        //default f in case of
        int d_frequency = 100;
        this->declare_parameter<int>("frequency", d_frequency);
        int frequency = this->get_parameter("frequency").as_int();

        publisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        timer = this->create_wall_timer(
            std::chrono::milliseconds(frequency),
            std::bind(&WebcamPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        cv::Mat frame;
        capture >> frame;

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture an image");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher->publish(*msg);
    }

    cv::VideoCapture capture;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebcamPublisher>());
    rclcpp::shutdown();
    return 0;
}