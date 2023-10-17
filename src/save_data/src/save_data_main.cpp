// Copyright 2023 xFranv8
#include <memory>

#include "save_data/SaveDataNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);

    auto savedata_node = std::make_shared<save_data::SaveDataNode>();
    rclcpp:spin(savedata_node);

    rclcpp::shutdown();
    
}