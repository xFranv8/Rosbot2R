// Copyright 2023 xFranv8
#include <vector>
#include <utility>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "save_data/SaveDataNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string.h>
#include <iostream>
#include <cstring>

namespace save_data {
using namespace std::chrono_literals;
using std::placeholders::_1;

SaveDataNode::SaveDataNode() : Node("save_data") {
    this->lidar_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "input_scan", rclcpp::SensorDataQoS(), 
        std::bind(&SaveDataNode::lidar_scan_callback, this, _1));

    this->image_sub_ = image_transport::create_subscription( this, 
        "input_image", 
        std::bind(&SaveDataNode::camera_scan_callback, this, _1), "raw", 
        rclcpp::SensorDataQoS().get_rmw_qos_profile());
    
    this->imu_scan_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "input_imu", rclcpp::SensorDataQoS(), 
        std::bind(&SaveDataNode::imu_scan_callback, this, _1));
    
    this->timer_ = create_wall_timer(50ms, std::bind(&SaveDataNode::control_cycle, this));

    this->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    this->serveraddr_in.sin_family = AF_INET;
    this->serveraddr_in.sin_port = htons(12345);
    this->serveraddr_in.sin_addr.s_addr = inet_addr(this->server_ip);

    if (connect(socket_fd, (struct sockaddr*)&server_ip, sizeof(server_ip)) < 0) {
        perror("Connection failed");
        return;
    }

    this->serverSock = socket(AF_INET, SOCK_STREAM, 0);

    if (this->serverSock < 0)
        RCLCPP_ERROR(get_logger(), "Failed to create socket.\n");
    
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(this->server_port);

    if (bind(serverSock, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0)
        RCLCPP_ERROR(get_logger(), "Failed to bind the socket.\n");
    
    if (listen(this->serverSock, 1) < 0)
        RCLCPP_ERROR(get_logger(), "Failed to listen for connections.\n");
    
    RCLCPP_ERROR(get_logger(), "Server started. Listening for incoming connections.\n");
    
    sockaddr_in clientAddress{};
    socklen_t clientAddressLength = sizeof(clientAddress);
    this->clientSock = accept(serverSock, (struct sockaddr*)&clientAddress, &clientAddressLength);
    
    if (clientSock < 0)
        RCLCPP_ERROR(get_logger(), "Failed to accept client connection.\n");

    RCLCPP_ERROR(get_logger(), "Client connected.\n");

    this->vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
}

SaveDataNode::~SaveDataNode() {
    close(this->socket_fd);
    close(this->serverSock);
}

void SaveDataNode::camera_scan_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        
        this->last_image_scan_ = image;
    } 
    catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

void SaveDataNode::lidar_scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg){
    this->last_lidar_scan_ = std::move(msg);
}

void SaveDataNode::imu_scan_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg){
    this->last_imu_scan_ = std::move(msg);
}

void SaveDataNode::send_lidar_data(const sensor_msgs::msg::LaserScan::UniquePtr & msg){
    std::vector<float> ranges = msg->ranges;
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;

    std::string data = "";

    for (int i = 0; i < ranges.size(); i++) {
       data += std::to_string(ranges[i]) + ",";
    }

    data += "\n";
    data += std::to_string(angle_min) + ",";
    data += std::to_string(angle_max);

    this->send_with_len(data);

    // send(this->socket_fd, data.c_str(), data.size(), 0);
}

void SaveDataNode::send_camera_data(cv::Mat image){
    std::vector<uchar> jpeg_data;
    cv::imencode(".jpg", image, jpeg_data);

    uint32_t image_len = htons(jpeg_data.size());
    uint32_t network_image_len = htons(image_len);

    char buffer[4 + image_len];
    memcpy(buffer, &network_image_len, sizeof(network_image_len));
    memcpy(buffer + sizeof(network_image_len), jpeg_data.data(), image_len);

    send(this->socket_fd, buffer, sizeof(buffer), 0);

    // send(this->socket_fd, jpeg_data.data(), jpeg_data.size(), 0);
}

void SaveDataNode::send_imu_data(const sensor_msgs::msg::Imu::ConstSharedPtr msg){
    std::string data = "";

    data += std::to_string(msg->orientation.x) + ",";
    data += std::to_string(msg->orientation.y) + ",";
    data += std::to_string(msg->orientation.z) + ",";
    data += std::to_string(msg->orientation.w) + ",";

    data += "\n";

    data += std::to_string(msg->angular_velocity.x) + ",";
    data += std::to_string(msg->angular_velocity.y) + ",";
    data += std::to_string(msg->angular_velocity.z) + ",";

    data += "\n";

    data += std::to_string(msg->linear_acceleration.x) + ",";
    data += std::to_string(msg->linear_acceleration.y) + ",";
    data += std::to_string(msg->linear_acceleration.z) + ",";

    this->send_with_len(data);

    // send(this->socket_fd, data.c_str(), data.size(), 0);
}

void SaveDataNode::send_cmd(std::vector<float> cmd){
    auto motor_msg = std::make_shared<geometry_msgs::msg::Twist>();
    motor_msg->linear.x = (float) cmd[0];
    motor_msg->angular.z = (float) cmd[1];

    RCLCPP_ERROR(get_logger(), "Linear: ", motor_msg->linear.x, "\n");
    RCLCPP_ERROR(get_logger(), "Angular: ", motor_msg->angular.z, "\n");

    this->vel_pub_->publish(*motor_msg);
}

void SaveDataNode::send_with_len(std::string data){
    uint32_t data_len = htons(data.size());

    char buffer[4 + data.size()];
    memcpy(buffer, &data_len, sizeof(data_len));
    memcpy(buffer + sizeof(data_len), data.c_str(), data.size());

    send(this->socket_fd, buffer, sizeof(buffer), 0);
}

void SaveDataNode::control_cycle(){
    if ((this->last_image_scan_.data == nullptr) || (this->last_lidar_scan_ == nullptr) || (this->last_imu_scan_ == nullptr))
        return;
    
    float buffer[2];
    ssize_t bytesRead = recv(clientSock, buffer, sizeof(buffer), 0);

    if (bytesRead == 0) {
        RCLCPP_ERROR(get_logger(), "Client disconnected.\n");
    } else if (bytesRead < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to receive message.\n");
    } else {
        float v = buffer[0];
        float w = buffer[1];

        std::vector<float> cmd = {v, w};
        this->send_cmd(cmd);

        // Pack the data as double-precision floats
        std::string packed_data(reinterpret_cast<const char*>(cmd.data()), cmd.size() * sizeof(double));

        this->send_with_len(packed_data);

        //send(this->socket_fd, cmd.data(), cmd.size(), 0);
    }

    this->send_camera_data(this->last_image_scan_);
    this->send_lidar_data(this->last_lidar_scan_);
    this->send_imu_data(this->last_imu_scan_);
}
} // namespace save_data
