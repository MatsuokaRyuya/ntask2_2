#include "ntask2_2/ntask2_2_joy.hpp"

JoySubscriber::JoySubscriber() : Node("joy_subscriber") 
{
    // Joyトピックの購読
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>( 
        "/joy", 10, std::bind(&JoySubscriber::joyCallback, this, std::placeholders::_1));

    // Dynamixelモーター制御用の速度コマンドパブリッシャー
    speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/dynamixel_motor/speed_command", 10);

    // ラックアンドピニオン位置制御用のパブリッシャー
    position_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/rack_position/command", 10);

    RCLCPP_INFO(this->get_logger(), "JoySubscriber has been started.");
}

void JoySubscriber::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{
    // 左スティックの上下 (axes[1]) を速度として使用
    double speed = msg->axes[1] * 300.0;  // スティックの上下を速度に変換（任意調整可能）

    // 右スティックの横移動 (axes[3]) をラックの位置に変換
    double right_stick_input = msg->axes[3];
    double max_rack_displacement = 10 ; // 最大ラック移動量（mm）
    double rack_position = right_stick_input * max_rack_displacement; // ラックの位置（mm）

    // 速度コマンドメッセージを作成
    auto speed_msg = std::make_shared<std_msgs::msg::Float64>();
    speed_msg->data = speed;

    // ラックの位置コマンドメッセージを作成
    auto position_msg = std::make_shared<std_msgs::msg::Float64>();
    position_msg->data = rack_position;

    // トピックにメッセージを送信
    speed_publisher_->publish(*speed_msg);
    position_publisher_->publish(*position_msg);

    RCLCPP_INFO(this->get_logger(), "Published speed: %.2f, Rack position: %.2f mm", speed, rack_position);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoySubscriber>());
    rclcpp::shutdown();
    return 0;
}


/*#include "ntask2_2/ntask2_2_joy.hpp"

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
    double right_stick_input = msg->axes[3];
    double max_rack_displacement = 20.0; // 最大ラック移動量（mm）
    double rack_position = right_stick_input * max_rack_displacement; // ラックの位置（mm）

    // ラックの位置をDynamixelの位置コマンドとして送信
    // ここでラックの位置をステップ数に変換する
    double steps_per_mm = 4096.0 / 100.53;  // 1回転で10mm移動し、4096ステップ
    int32_t position = static_cast<int32_t>(rack_position * steps_per_mm); // ラック位置をステップ数に変換

    // モーター制御メッセージを作成
    auto position_msg = std::make_shared<std_msgs::msg::Float64>();
    position_msg->data = position;

    // Dynamixelモーターにラックの位置をコマンドとして送信
    publisher_->publish(*position_msg);

    RCLCPP_INFO(this->get_logger(), "Published speed: %.2f, Rack position: %.2f", speed, rack_position);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoySubscriber>());
    rclcpp::shutdown();
    return 0;
}



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

    //右スティックの横移動(axes[3])を使用してラックアンドピニオンの位置を制御
    double right_stick_input = msg->axes[3];

    double max_rack_displacement = 20;
    double rack_position = right_stick_input * max_rack_displacement;

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