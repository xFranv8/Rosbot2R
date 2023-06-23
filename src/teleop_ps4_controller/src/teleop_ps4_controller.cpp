//
// Created by xfranv8 on 14/06/23.
//
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

using namespace std::chrono_literals;

class TeleopPS4Controller : public rclcpp::Node{
public:
    TeleopPS4Controller() : Node("teleop_ps4_controller"){
        server_port = 6278;

        serverSock = socket(AF_INET, SOCK_STREAM, 0);

        if (serverSock < 0) {
            std::cerr << "Failed to create socket." << std::endl;
        }

        serverAddress.sin_family = AF_INET;
        serverAddress.sin_addr.s_addr = INADDR_ANY;
        serverAddress.sin_port = htons(server_port);
        if (bind(serverSock, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0)
            std::cerr << "Failed to bind the socket." << std::endl;

        if (listen(serverSock, 1) < 0)
            std::cerr << "Failed to listen for connections." << std::endl;

        std::cout << "Server started. Listening for incoming connections..." << std::endl;

        sockaddr_in clientAddress{};
        socklen_t clientAddressLength = sizeof(clientAddress);
        clientSock = accept(serverSock, (struct sockaddr*)&clientAddress, &clientAddressLength);
        if (clientSock < 0)
            std::cerr << "Failed to accept client connection." << std::endl;

        std::cout << "Client connected." << std::endl;

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(
                50ms, std::bind(&TeleopPS4Controller::timer_callback, this));

    }
    ~TeleopPS4Controller(){
        close(serverSock);
    }

    void send_cmd(std::vector<float> cmd){
        auto motor_msg = std::make_shared<geometry_msgs::msg::Twist>();
        motor_msg->linear.x = (float) cmd[0];
        motor_msg->angular.z = (float) cmd[1];
        std::cout << "linear: " << motor_msg->linear.x << std::endl;
        std::cout << "angular: " << motor_msg->angular.z << std::endl;
        this->publisher_->publish(*motor_msg);
    }

    void timer_callback(){
        float buffer[2];
        ssize_t bytesRead = recv(clientSock, buffer, sizeof(buffer), 0);
        if (bytesRead == 0) {
            std::cout << "Client disconnected." << std::endl;
        } else if (bytesRead < 0) {
            std::cerr << "Failed to receive message." << std::endl;
        } else {
            float v = buffer[0];
            float w = buffer[1];
            std::cout << "V: " << v << std::endl;
            std::cout << "W: " << w << std::endl;
            std::cout << "----------------" << std::endl;

            std::vector<float> cmd = {v, w};
            send_cmd(cmd);

        }
    }

private:
    int server_port;
    int serverSocket;
    int clientSock;
    sockaddr_in serverAddress{};
    sockaddr_in clientAddr;
    int serverSock;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopPS4Controller>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
