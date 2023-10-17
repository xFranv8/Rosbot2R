// Copyright 2023 xFranv8

#include <utility>
#include "fsm_bumpgo/BumpGoNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace fsm_bumpgo {
using namespace std::chrono_literals;
using std::placeholders::_1;

BumpGoNode::BumpGoNode() : Node("bump_go"), state_(FORWARD) {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "input_scan", rclcpp::SensorDataQoS(), 
        std::bind(&BumpGoNode::scan_callback, this, _1));
    
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
    timer_ = create_wall_timer(50ms, std::bind(&BumpGoNode::control_cycle, this));

    state_ts_ = now();
}


void BumpGoNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg) {
    last_scan_ = std::move(msg);
}


void BumpGoNode::control_cycle(){
    if (last_scan_ == nullptr) {
        return;
    }

    geometry_msgs::msg::Twist out_vel;

    switch (state_) {
        case FORWARD:
            out_vel.linear.x = SPEED_LINEAR;

            if (check_forward_2_stop()) {
                go_state(STOP);
            }

            if (check_forward_2_turn()) {
                go_state(TURN);
            }

            break;
        
        case TURN:
            this->turning_time_ = rclcpp::Duration::from_seconds(get_time_open_cycle(last_scan_->ranges) + 1.0);
            out_vel.angular.z = SPEED_ANGULAR;

            if (check_turn_2_forward()) {
                go_state(FORWARD);
            }

            break;
        
        case STOP:
            if (check_stop_2_forward()) {
                go_state(FORWARD);
            }

            break;
    }

    vel_pub_->publish(out_vel);
}


void BumpGoNode::go_state(int new_state) {
    state_ = new_state;
    state_ts_ = now();
}


bool BumpGoNode::check_forward_2_turn() {
    std::vector<float> ranges = last_scan_->ranges;
    
    this->frontal_distance_ = ranges[0];

    return this->frontal_distance_ < OBSTACLE_DISTANCE;
}


bool BumpGoNode::check_forward_2_stop() {
    auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);

    return elapsed > SCAN_TIMEOUT;
}


bool BumpGoNode::check_stop_2_forward() {
    auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);

    return elapsed < SCAN_TIMEOUT;
}


bool BumpGoNode::check_turn_2_forward() {
    auto elapsed = now() - state_ts_;

    return elapsed > turning_time_;
}


float BumpGoNode::get_time_open_cycle(std::vector<float> ranges){
    float angle = 0;

    for (long unsigned int i = 0; i < ranges.size(); i++) {
        if (std::isinf(ranges[i])) {
            angle = i * last_scan_->angle_increment;

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Index: " << i << "\n");
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Turning angle: " << angle << "\n");
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Distance: " << ranges[i] << "\n");
            break;
        }
    }

    return angle / SPEED_ANGULAR;
}

} // namespace fsm_bumpgo