#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include <zbar.h>

class QRCodeReader : public rclcpp::Node
{
public:
    QRCodeReader() : Node("qr_code_reader")
    {
        // Correction: Utiliser std_msgs::msg::String
        publisher_ = this->create_publisher<std_msgs::msg::String>("qr_code_content", 10);
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10, std::bind(&QRCodeReader::image_callback, this, std::placeholders::_1));
            
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
            try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat cv_image = cv_ptr->image;
            cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2GRAY);
            zbar::Image zbar_image(cv_image.cols, cv_image.rows, "Y800", cv_image.data, cv_image.cols * cv_image.rows);

            scanner_.scan(zbar_image);

            for(auto symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
                if(symbol->get_type() == zbar::ZBAR_QRCODE) {
                    std_msgs::msg::String qr_content;
                    qr_content.data = symbol->get_data();
                    this->publisher_->publish(qr_content);
                }
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
    }

    zbar::ImageScanner scanner_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QRCodeReader>());
    rclcpp::shutdown();
    return 0;
}
