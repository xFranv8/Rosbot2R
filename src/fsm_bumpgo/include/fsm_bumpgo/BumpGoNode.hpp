// Copyright 2023 xFranv8

#ifndef BUMPGO__BUMPGONODE_HPP_
#define BUMPGO__BUMPGONODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace fsm_bumpgo {

using namespace std::chrono_literals;

class BumpGoNode : public rclcpp::Node{

public:
    BumpGoNode();

private:
    void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
    void control_cycle();

    static const int FORWARD = 0;
    static const int TURN = 1;
    static const int STOP = 2;
    int state_;
    rclcpp::Time state_ts_;

    void go_state(int new_state);
    bool check_forward_2_turn();
    bool check_forward_2_stop();
    bool check_turn_2_forward();
    bool check_stop_2_forward();
    float get_time_open_cycle(std::vector<float> ranges);

    
    const rclcpp::Duration SCAN_TIMEOUT {1s};
    rclcpp::Duration turning_time_ {2s};
    
    static constexpr float SPEED_LINEAR = 0.35f;
    static constexpr float SPEED_ANGULAR = 0.35f;
    static constexpr float OBSTACLE_DISTANCE = 0.8f;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
    float frontal_distance_;
    float left_distance_;
    float right_distance_;
};

}

#endif