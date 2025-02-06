#include "ntask2_2/ntask2_2_joy.hpp"

JoySubscriber::JoySubscriber() : Node("joy_subscriber") 
{
    // Joyトピックの購読
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>( 
        "/joy", 10, std::bind(&JoySubscriber::joyCallback, this, std::placeholders::_1));

    // Dynamixelモーターの位置制御用パブリッシャー
    position_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/dynamixel_motor/command", 10);
    
    // Dynamixelモーターの速度制御用パブリッシャー
    speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/dynamixel_motor/speed_command", 10);

    RCLCPP_INFO(this->get_logger(), "JoySubscriber has been started.");
}

void JoySubscriber::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{
    // ジョイスティックの左スティック上下 (axes[1]) を速度に変換
    double speed = msg->axes[1] * 300.0;  // ここでのスケーリングは任意

    // 右スティック横方向 (axes[3]) を位置制御用入力値として利用
    double right_stick_input = msg->axes[3];

    // モーター速度制御メッセージを作成
    auto speed_msg = std::make_shared<std_msgs::msg::Float64>();
    speed_msg->data = speed;

    // モーター位置制御メッセージを作成
    auto position_msg = std::make_shared<std_msgs::msg::Float64>();
    position_msg->data = right_stick_input;

    // トピックに速度と位置をそれぞれパブリッシュ
    speed_publisher_->publish(*speed_msg);
    position_publisher_->publish(*position_msg);

    RCLCPP_INFO(this->get_logger(), "Published speed: %.2f, Rack position: %.2f", speed, right_stick_input);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoySubscriber>());
    rclcpp::shutdown();
    return 0;
}
