#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "data_collection/ControlNode.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.cpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace data_collection
{
using std::placeholders::_1;

CameraNode::CameraNode() : Node("camera") {
    image_sub_ = image_transport::create_subscription(
        this, "input_image", std::bind(&CameraNode::image_callback, this, _1),
        "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    
    clientSocket = socket(AF_INET, SOCK_DGRAM, 0);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(8080);
    serverAddr.sin_addr.s_addr = inet_addr("10.0.81.65");

    connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
}

CameraNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;
    std::vector<uchar> jpeg_image;
    cv::imencode(".jpg", image, jpeg_image);

    send(this->clientSocket, jpeg_image.data(), jpeg_image.size(), 0);    
}
} // namespace data_collection