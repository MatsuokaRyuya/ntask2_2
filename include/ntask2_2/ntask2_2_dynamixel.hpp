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
     // モーターの現在位置を取得
    int32_t getMotorPosition();

    // モーターの位置を設定
    void setMotorPosition(int32_t position);

    // モーターの初期位置を設定
    void initializeMotorPosition();
    // コールバック関数
    void speedCommandCallback(const std_msgs::msg::Float64::SharedPtr msg);       // 速度制御用コールバック
    void positionCommandCallback(const std_msgs::msg::Float64::SharedPtr msg);    // 位置制御用コールバック

    // Dynamixelの制御関数
    void enableTorque(uint8_t id);                // トルクを有効化
    void disableTorque(uint8_t id);               // トルクを無効化
    void setPositionControlMode(uint8_t id);      // 位置制御モードの設定
    void sendVelocityCommand(uint8_t id, int32_t velocity); // 速度コマンド送信
    void sendPositionCommand(uint8_t id, int32_t position); // 位置コマンド送信
    void setLED(uint8_t id, bool state);          // LEDの設定

    // メンバ変数
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;          // 速度制御コマンドの購読
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rack_position_subscription_; // 位置制御コマンドの購読
    dynamixel::PortHandler *portHandler_;             // Dynamixelポートハンドラ
    dynamixel::PacketHandler *packetHandler_;         // Dynamixelパケットハンドラ
    std::vector<uint8_t> motor_ids_;
    uint8_t motor_id_ = 3;           // 使用するモーターのIDリスト
};

#endif // NTASK2_2_DYNAMIXEL_HPP



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