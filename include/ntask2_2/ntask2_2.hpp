#ifndef NTASK2_2_HPP
#define NTASK2_2_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <numeric>
#include <string>
#include <vector>
#include "std_msgs/msg/float64.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h" // Dynamixel SDKのヘッダー

class JoySubscriber : public rclcpp::Node 
{
public:
    JoySubscriber();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_; // Joyトピックの購読
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;      // Dynamixel制御用パブリッシャー
};



class DynamixelController : public rclcpp::Node 
{
public:
    DynamixelController();
    ~DynamixelController();

private:
    void commandCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void enableTorque(uint8_t id);
    void disableTorque(uint8_t id);
    void sendVelocityCommand(uint8_t id, int32_t velocity);

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_; // トピック購読
    dynamixel::PortHandler *portHandler_;  // Dynamixelポートハンドラ
    dynamixel::PacketHandler *packetHandler_; // Dynamixelパケットハンドラ
    std::vector<uint8_t> motor_ids_; // 使用するモーターのID
};


#endif