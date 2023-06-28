// Copyright 2023 xFranv8

#include <utility>
#include "fsm_bumpgo/BumpGoNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace fsm_bumpgo {
using namespace std::chrono_literals;
using std::placeholders::_1;

BumpGoNode::BumpGoNode() : Node("bump_go"), state_(FORWARD) {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "input_scan", rclcpp::SensorDataQoS(), 
        std::bind(&BumpGoNode::scan_callback, this, _1));
    
    vel_pub_ =create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
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

            if (check_forward_2_back()) {
                go_state(BACK);
            }

            break;

        case BACK:
            out_vel.linear.x = -SPEED_LINEAR;

            if (check_back_2_turn()) {
                go_state(TURN);
            }

            break;
        
        case TURN:
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


bool BumpGoNode::check_forward_2_back() {
    size_t pos = last_scan_->ranges.size() / 2;

    return last_scan_->ranges[pos] < OBSTACLE_DISTANCE;
}


bool BumpGoNode::check_forward_2_stop() {
    auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);

    return elapsed > SCAN_TIMEOUT;
}


bool BumpGoNode::check_stop_2_forward() {
    auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);

    return elapsed < SCAN_TIMEOUT;
}


bool BumpGoNode::check_back_2_turn() {
    auto elapsed = now() - state_ts_;

    return elapsed > BACKING_TIME;
}


bool BumpGoNode::check_turn_2_forward() {
    auto elapsed = now() - state_ts_;

    return elapsed > TURNING_TIME;
}

} // namespace fsm_bumpgo