#ifndef NTASK2_2_DYNAMIXEL_HPP
#define NTASK2_2_DYNAMIXEL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h" // Dynamixel SDKのヘッダー
#include <vector>

class DynamixelController : public rclcpp::Node 
{
public:
    DynamixelController();
    ~DynamixelController();

private:
    void commandCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void setRackPositionCallback(const std_msgs::msg::Float64::SharedPtr msg);

    void enableTorque(uint8_t id);
    void disableTorque(uint8_t id);
    void setPositionControlMode(uint8_t id);
    void sendVelocityCommand(uint8_t id, int32_t velocity);
    void sendPositionCommand(uint8_t id, int32_t position);
    void setLED(uint8_t id, bool state);

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;  // 速度制御の購読
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rack_position_subscription_; // 位置制御の購読
    dynamixel::PortHandler *portHandler_;  // Dynamixelポートハンドラ
    dynamixel::PacketHandler *packetHandler_; // Dynamixelパケットハンドラ
    std::vector<uint8_t> motor_ids_; // 使用するモーターのID
};

#endif



/*
#ifndef NTASK2_2_DYNAMIXEL_HPP
#define NTASK2_2_DYNAMIXEL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <numeric>
#include <string>
#include <vector>
#include "std_msgs/msg/float64.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h" // Dynamixel SDKのヘッダー

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

    // LED制御用の新しい関数
    void setLED(uint8_t id, bool state);

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_; // トピック購読
    dynamixel::PortHandler *portHandler_;  // Dynamixelポートハンドラ
    dynamixel::PacketHandler *packetHandler_; // Dynamixelパケットハンドラ
    std::vector<uint8_t> motor_ids_; // 使用するモーターのID
};

#endif



#ifndef NTASK2_2_DYNAMIXEL_HPP
#define NTASK2_2_DYNAMIXEL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <numeric>
#include <string>
#include <vector>
#include "std_msgs/msg/float64.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h" // Dynamixel SDKのヘッダー


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
*/