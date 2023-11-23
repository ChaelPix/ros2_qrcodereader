#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"

#include <string>
#include <map>

class ColorReader : public rclcpp::Node
{
public:
    ColorReader() : Node("color_reader")
    {
        //Publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("color", 10);

        //Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10, std::bind(&ColorReader::image_callback, this, std::placeholders::_1));

    }

private:

#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <limits>
#include <cmath>

std::map<std::string, cv::Scalar> knownColors = {
    {"Red", cv::Scalar(0, 0, 255)},
    {"Blue", cv::Scalar(255, 0, 0)},
    {"Green", cv::Scalar(0, 255, 0)},
    {"Yellow", cv::Scalar(0, 255, 255)}
    // Ajoutez d'autres couleurs ici si nécessaire
};

double colorDistance(const cv::Scalar& col1, const cv::Scalar& col2) {
    double r = col1.val[2] - col2.val[2];
    double g = col1.val[1] - col2.val[1];
    double b = col1.val[0] - col2.val[0];
    return sqrt(r * r + g * g + b * b);
}

std::string getClosestColorName(const cv::Scalar& color) {
    double minDist = std::numeric_limits<double>::max();
    std::string closestColor = "None";

    for (const auto& colorPair : knownColors) {
        double dist = colorDistance(color, colorPair.second);
        if (dist < minDist) {
            minDist = dist;
            closestColor = colorPair.first;
        }
    }
    return closestColor;
}

cv::Scalar image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_image = cv_ptr->image;

    cv::Mat hsv;
    cv::cvtColor(cv_image, hsv, cv::COLOR_BGR2HSV);

    // Seuils pour différentes couleurs
    // Rouge
    cv::Mat redMask1, redMask2;
    cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), redMask1);
    cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), redMask2);
    cv::Mat redMask = redMask1 | redMask2;

    // Bleu
    cv::Mat blueMask;
    cv::inRange(hsv, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), blueMask);

    // Vert
    cv::Mat greenMask;
    cv::inRange(hsv, cv::Scalar(50, 50, 50), cv::Scalar(70, 255, 255), greenMask);

    // Jaune
    cv::Mat yellowMask;
    cv::inRange(hsv, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), yellowMask);

    // Combinez tous les masques
    cv::Mat combinedMask = redMask | blueMask | greenMask | yellowMask;

    // Trouver les contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(combinedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        // Choisissez le plus grand contour
        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
            return cv::contourArea(c1) > cv::contourArea(c2);
        });

        cv::Rect roi = cv::boundingRect(contours[0]);
        cv::Scalar meanColor = cv::mean(cv_image(roi));

        // Obtenez le nom de la couleur la plus proche
        std::string colorName = getClosestColorName(meanColor);
        RCLCPP_INFO(this->get_logger(), colorName.c_str());
        return meanColor;  // Retournez le nom de la couleur
    } else {
        return cv::Scalar(0, 0, 0);  // Aucune couleur détectée
    }
}

    sensor_msgs::msg::Image image;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorReader>());
    rclcpp::shutdown();
    return 0;
}
