// Copyright 2023 xFranv8

#include <memory>

#include "fsm_bumpgo/BumpGoNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);

    auto bumpgo_node = std::make_shared<fsm_bumpgo::BumpGoNode>();
    rclcpp:spin(bumpgo_node);

    rclcpp::shutdown();
    
}