#include "ntask2_2/ntask2_2_dynamixel.hpp"

DynamixelController::DynamixelController() : Node("dynamixel_controller") 
{
    // 速度制御用のトピック購読
    speed_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/dynamixel_motor/speed_command", 10,
        std::bind(&DynamixelController::speedCallback, this, std::placeholders::_1));

    // ラックアンドピニオン位置制御用のトピック購読
    position_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/dynamixel_motor/command", 10,
        std::bind(&DynamixelController::positionCallback, this, std::placeholders::_1));
    
    // ボタンのコマンドを受け取る
    button_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/dynamixel_motor/button_command", 10, 
        std::bind(&DynamixelController::buttonCallback, this, std::placeholders::_1));

    portHandler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    motor_ids_ = {1, 2, 3}; // 使用するモーターのIDを設定
    if (portHandler_->setBaudRate(57600)) {
        RCLCPP_INFO(this->get_logger(), "Succeeded to change the baudrate!");
    } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to change the baudrate!");
    return;
}

    base_position_ = 0;  // 初期化
    // 各モーターの初期設定
    for (uint8_t id : motor_ids_) 
    {
        enableTorque(id);
        setVelocityControlMode(id);
        setLED(id, true); // 起動時にLEDを点灯
    }

    calibratePosition(3);
}

DynamixelController::~DynamixelController() 
{
    for (uint8_t id : motor_ids_) 
    {
        setLED(id, false); // 終了時にLEDを消灯
        disableTorque(id);
    }
    portHandler_->closePort();
}

void DynamixelController::speedCallback(const std_msgs::msg::Float64::SharedPtr msg) 
{
    double speed = msg->data;
    int32_t velocity_value = static_cast<int32_t>(speed);
    for (uint8_t id = 1; id <= 2; ++id) 
    {
        if(id == 1)
        {sendVelocityCommand(id, velocity_value);}
        else
        {sendVelocityCommand(id, -velocity_value);}
    }
    RCLCPP_INFO(this->get_logger(), "Set velocity: %.2f", speed);
}

void DynamixelController::positionCallback(const std_msgs::msg::Float64::SharedPtr msg) 
{
    double joystick_input = msg->data; // -1 から 1 までの値
    double deadzone = 0.05;
    double max_speed = 100;  // 最大速度 (Dynamixel の設定に応じて調整)

    if (std::abs(joystick_input) < deadzone) 
    {
        joystick_input = 0.0;
    }

    int32_t velocity = static_cast<int32_t>(joystick_input * max_speed);
    
    int32_t current_position = readPosition(3); // 現在位置を取得
    if (current_position == -1) return; // 位置取得失敗時は処理しない

    // 移動範囲制限
    const int32_t min_position = base_position_ - 200;
    const int32_t max_position = base_position_ + 200;

    // 限界位置に到達したら速度を 0 にする
    if ((current_position <= min_position && -velocity < 0) || 
        (current_position >= max_position && -velocity > 0)) 
    {
        velocity = 0;
    }

    sendVelocityCommand(3, -velocity); // 速度指令を送信

    RCLCPP_INFO(this->get_logger(), "Joystick: %.2f, Speed: %d, Position: %d", 
                joystick_input, velocity, current_position);

    // ジョイスティックを戻したら基準位置へゆっくり戻る
    if (joystick_input == 0) 
    {
        int32_t return_speed = (base_position_ - current_position) * 0.15; // 緩やかに戻る
        return_speed = std::clamp(return_speed, static_cast<int32_t>(-max_speed), static_cast<int32_t>(max_speed));
        sendVelocityCommand(3, return_speed);
    }
}

void DynamixelController::buttonCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (msg->data == 1) // ボタン 5 (L1 / LB) が押されたら
    {
        int32_t new_base = readPosition(3);
        if (new_base != -1)
        {
            base_position_ = new_base;
            RCLCPP_INFO(this->get_logger(), "Calibration complete! New base position: %d", base_position_);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Calibration failed: Unable to read position.");
        }
    }
}

void DynamixelController::enableTorque(uint8_t id)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 64, 1, &dxl_error);
}

void DynamixelController::disableTorque(uint8_t id)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 64, 0, &dxl_error);
}

int32_t DynamixelController::readPosition(uint8_t id)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result;
    uint32_t dxl_present_position = 0; // 現在位置

    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, 132, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read position for ID %d: %s",
                    id, packetHandler_->getTxRxResult(dxl_comm_result));
        return -1; // エラー時は -1 を返す
    }

    return static_cast<int32_t>(dxl_present_position);
}

void DynamixelController::calibratePosition(uint8_t id)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result;
    uint32_t dxl_present_position = 0; // 位置データ格納用

    // 4バイトのデータを読み取る（現在位置取得）
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, 132, &dxl_present_position, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read position for ID %d: %s",
                     id, packetHandler_->getTxRxResult(dxl_comm_result));
    } 
    else 
    {
        base_position_ = static_cast<int32_t>(dxl_present_position); // 基準位置を保存
        RCLCPP_INFO(this->get_logger(), "Calibration complete. Base position for ID %d: %d", id, base_position_);
    }
}

void DynamixelController::setVelocityControlMode(uint8_t id) 
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 11, 1, &dxl_error); // 1は速度制御モード
}

void DynamixelController::sendVelocityCommand(uint8_t id, int32_t velocity) 
{
    uint8_t dxl_error = 0;
    // 修正: motor_id ループではなく、指定された id のみを使用する
    int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, 104, velocity, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send velocity command to motor ID %d: %s", 
                     id, packetHandler_->getTxRxResult(dxl_comm_result));
    }
}

void DynamixelController::setLED(uint8_t id, bool state) 
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 65, state ? 1 : 0, &dxl_error);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}
