#include "ntask2_2/ntask2_2_joy.hpp"

JoySubscriber::JoySubscriber() : Node("joy_subscriber") 
{
    // Joyトピックの購読
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>( 
        "/joy", 10, std::bind(&JoySubscriber::joyCallback, this, std::placeholders::_1));

    // Dynamixelモーター制御用のパブリッシャーを作成
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/dynamixel_motor/command", 10);

    RCLCPP_INFO(this->get_logger(), "JoySubscriber has been started.");
}

void JoySubscriber::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{
    // ジョイスティックのボタンと軸の値をログに出力
    RCLCPP_INFO(this->get_logger(), "Axes: [%s]", 
        std::accumulate(msg->axes.begin(), msg->axes.end(), std::string(),
                        [](std::string a, float b) { return a + (a.empty() ? "" : ", ") + std::to_string(b); }).c_str());
    RCLCPP_INFO(this->get_logger(), "Buttons: [%s]", 
        std::accumulate(msg->buttons.begin(), msg->buttons.end(), std::string(),
                        [](std::string a, int b) { return a + (a.empty() ? "" : ", ") + std::to_string(b); }).c_str());

    // 左スティックの上下 (axes[1]) をDynamixelモーターの速度として使用
    double speed = msg->axes[1] * 300.0;  // スティックの上下を速度に変換（ここは任意に調整）

    // 右スティックの横移動 (axes[3]) をラックアンドピニオンの位置に変換
    double right_stick_input = msg->axes[3];//-1から1まで

    /*
    double max_rack_displacement = 20.0; // 最大ラック移動量（mm）
    double rack_position = right_stick_input * max_rack_displacement; // ラックの位置（mm）

    // ラックの位置をDynamixelの位置コマンドとして送信
    // ここでラックの位置をステップ数に変換する
    double steps_per_mm = 4096.0 / 100.53;  // 1回転で10mm移動し、4096ステップ
    int32_t position = static_cast<int32_t>(rack_position * steps_per_mm); // ラック位置をステップ数に変換
    */

    // モーター制御メッセージを作成
    auto position_msg = std::make_shared<std_msgs::msg::Float64>();
    position_msg->data = right_stick_input;

    // Dynamixelモーターにラックの位置をコマンドとして送信
    publisher_->publish(*position_msg);

    RCLCPP_INFO(this->get_logger(), "Published speed: %.2f, Rack position: %.2f", speed, right_stick_input);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoySubscriber>());
    rclcpp::shutdown();
    return 0;
}
