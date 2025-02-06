#include "ntask2_2/ntask2_2_joy.hpp"

JoySubscriber::JoySubscriber() : Node("joy_subscriber") 
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>( 
        "/joy", 10, std::bind(&JoySubscriber::joyCallback, this, std::placeholders::_1));

    position_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/dynamixel_motor/command", 10);
    speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/dynamixel_motor/speed_command", 10);
    button_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/dynamixel_motor/button_command", 10);
    
    RCLCPP_INFO(this->get_logger(), "JoySubscriber has been started.");
}

void JoySubscriber::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{
    double speed = msg->axes[1] * 70.0;
    double right_stick_input = msg->axes[3];

    auto speed_msg = std::make_shared<std_msgs::msg::Float64>();
    speed_msg->data = speed;

    auto position_msg = std::make_shared<std_msgs::msg::Float64>();
    position_msg->data = right_stick_input;

    auto button_msg = std::make_shared<std_msgs::msg::Int32>(); // Int32に変更
    button_msg->data = msg->buttons[5]; // ボタン5 (L1 / LB)

    speed_publisher_->publish(*speed_msg);
    position_publisher_->publish(*position_msg);
    button_publisher_->publish(*button_msg);

    RCLCPP_INFO(this->get_logger(), "Published speed: %.2f, Rack position: %.2f, Button: %d", speed, right_stick_input, msg->buttons[5]);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoySubscriber>());
    rclcpp::shutdown();
    return 0;
}