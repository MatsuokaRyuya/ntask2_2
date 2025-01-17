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
    double speed = msg->axes[1] * 300.0;

    // モーター制御メッセージを作成
    auto command_msg = std::make_shared<std_msgs::msg::Float64>();
    command_msg->data = speed;

    // Dynamixelモーターにコマンドを送信
    publisher_->publish(*command_msg);

    RCLCPP_INFO(this->get_logger(), "Published speed: %.2f", speed);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoySubscriber>());
    rclcpp::shutdown();
    return 0;
}











/*

class JoySubscriber : public rclcpp::Node 
{
public:
    JoySubscriber() : Node("joy_subscriber") 
    {
        // Joyトピックの購読
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoySubscriber::joyCallback, this, std::placeholders::_1));

        // Dynamixelモーター制御用のパブリッシャーを作成
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/dynamixel_motor/command", 10);

        RCLCPP_INFO(this->get_logger(), "JoySubscriber has been started.");
    }

private:
    // Joyメッセージを受信したときのコールバック関数
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
    {
        // ジョイスティックのボタンと軸の値をログに出力
        RCLCPP_INFO(this->get_logger(), "Axes: [%s]", 
            std::accumulate(msg->axes.begin(), msg->axes.end(), std::string(),
                            [](std::string a, float b) { return a + (a.empty() ? "" : ", ") + std::to_string(b); }).c_str());
        RCLCPP_INFO(this->get_logger(), "Buttons: [%s]", 
            std::accumulate(msg->buttons.begin(), msg->buttons.end(), std::string(),
                            [](std::string a, int b) { return a + (a.empty() ? "" : ", ") + std::to_string(b); }).c_str());

        // 左スティックの上下 (axes[1]) をDynamixelモーターの速度として使用
        double speed = msg->axes[1] * 100.0; // スケーリング (例: -100 ~ 100)

        // モーター制御メッセージを作成
        auto command_msg = std::make_shared<std_msgs::msg::Float64>();
        command_msg->data = speed;

        // Dynamixelモーターにコマンドを送信
        publisher_->publish(*command_msg);

        RCLCPP_INFO(this->get_logger(), "Published speed: %.2f", speed);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_; // Joyトピックの購読
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;      // Dynamixel制御用パブリッシャー
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoySubscriber>());
    rclcpp::shutdown();
    return 0;
}
*/