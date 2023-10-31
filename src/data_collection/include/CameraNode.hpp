#ifndef CAMERA_NODE_HPP_
#define CAMERA_NODE_HPP_

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace data_collection
{
class CameraNode : public rclcpp::Node 
{
public:
    CameraNode();

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

private:
    image_transport::Subscriber image_sub_;

    int clientSocket;
    struct sockaddr_in serverAddr;
};

} // namespace data_collection

#endif