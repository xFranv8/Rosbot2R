#include <memory>

#include "data_collection/CameraNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);

    auto camera = std::make_shared<data_collection::CameraNode>();
    rclcpp:spin(camera);

    rclcpp::shutdown();
    
}