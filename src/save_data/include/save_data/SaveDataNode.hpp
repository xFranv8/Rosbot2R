// Copyright 2023 xFranv8

#ifndef SAVE_DATA__SAVEDATANODE_HPP_
#define SAVE_DATA__SAVEDATANODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#include "opencv2/opencv.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

namespace save_data {

using namespace std::chrono_literals;

class SaveDataNode : public rclcpp::Node {

public:
    SaveDataNode();
    ~SaveDataNode();

private:
    void lidar_scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
    void camera_scan_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    void imu_scan_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void control_cycle();

    void send_lidar_data(const sensor_msgs::msg::LaserScan::UniquePtr & msg);
    void send_camera_data(const cv::Mat & image);
    void send_imu_data(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    void send_cmd(std::vector<float> cmd);
    void send_with_len(std::string data);
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub_;
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Mat last_image_scan_;
    sensor_msgs::msg::LaserScan::UniquePtr last_lidar_scan_;
    sensor_msgs::msg::Imu::ConstSharedPtr last_imu_scan_;

    int socket_fd;
    sockaddr_in serveraddr_in{};
    const char * server_ip = "10.0.81.65";

    int server_port = 6278;
    int clientSock;
    sockaddr_in serverAddress{};
    int serverSock;
};

}

#endif