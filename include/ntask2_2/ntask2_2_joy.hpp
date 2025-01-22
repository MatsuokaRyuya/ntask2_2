#ifndef NTASK2_2_JOY_HPP
#define NTASK2_2_JOY_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <numeric>
#include <string>
#include <vector>
#include "std_msgs/msg/float64.hpp"

class JoySubscriber : public rclcpp::Node 
{
public:
    JoySubscriber();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_; // Joyトピックの購読
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;      // Dynamixel制御用パブリッシャー
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_publisher_;
};

#endif