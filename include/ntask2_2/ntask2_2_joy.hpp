#ifndef NTASK2_2_JOY_HPP
#define NTASK2_2_JOY_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <string>
#include <vector>
#include <numeric>

class JoySubscriber : public rclcpp::Node 
{
public:
    JoySubscriber();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // Joyトピックの購読
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    // 速度制御用パブリッシャー
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;

    // 位置制御用パブリッシャー
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_publisher_;
};

#endif // NTASK2_2_JOY_HPP
