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
        //Config Zbar Scanner 
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

        //Publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("qr_code_content", 10);

        //Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10, std::bind(&QRCodeReader::image_callback, this, std::placeholders::_1));



    }

private:

    void read_qr_code()
    {
        

        try {
            //Ros Img to OpenCv image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
            cv::Mat cv_image = cv_ptr->image;
            cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2GRAY);

            //OpenCv image to ZBar image
            zbar::Image zbar_image(cv_image.cols, cv_image.rows, "Y800", cv_image.data, cv_image.cols * cv_image.rows);

            //Scan image
            scanner_.scan(zbar_image);

            bool isSucess = false;
            for(auto symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {

                if(symbol->get_type() == zbar::ZBAR_QRCODE) {
                    //Get qr code content and publishb it
                    std_msgs::msg::String qr_content;
                    qr_content.data = symbol->get_data();
        
                    isSucess = true;
                    
                }
            }

            if(!isSucess)
            {
                std_msgs::msg::String qr_content;
                qr_content.data = "FAIL : No QR Code Detected on the Image";
               
            }

        } catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }

    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image = *msg;
    }

    sensor_msgs::msg::Image image;

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
