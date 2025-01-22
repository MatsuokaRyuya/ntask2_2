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

    portHandler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB1");
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (portHandler_->openPort()) 
    {
        RCLCPP_INFO(this->get_logger(), "Port opened successfully.");
    } 
    else 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the port.");
        rclcpp::shutdown();
    }

    if (portHandler_->setBaudRate(57600)) 
    {
        RCLCPP_INFO(this->get_logger(), "Baudrate set successfully.");
    } 
    else 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate.");
        rclcpp::shutdown();
    }

    motor_ids_ = {1, 2, 3}; // 使用するモーターのIDを設定

    // 各モーターの初期設定
    for (uint8_t id : motor_ids_) 
    {
        enableTorque(id);
        if (id == 3) 
        {
            setPositionControlMode(id); // ID 3を位置制御モードに設定
            sendPositionCommand(id, 1400); // 初期位置を1400に設定
        }
        setLED(id, true); // 起動時にLEDを点灯
    }
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
        sendVelocityCommand(id, velocity_value);
    }
    RCLCPP_INFO(this->get_logger(), "Set velocity: %.2f", speed);
}

void DynamixelController::positionCallback(const std_msgs::msg::Float64::SharedPtr msg) 
{
    double displacement_mm = msg->data; // -1から1まで
    double steps_per_mm = 100; // 1mmあたりのステップ数
    int32_t position_value = static_cast<int32_t>(1400 + displacement_mm * steps_per_mm);

    // 範囲を1400±500に制限
    const int32_t min_position = 900;
    const int32_t max_position = 1900;

    position_value = std::clamp(position_value, min_position, max_position);

    sendPositionCommand(3, position_value); // ID 3のモーターを制御
    RCLCPP_INFO(this->get_logger(), "Rack position command: %.2f mm, steps: %d", displacement_mm, position_value);
}

void DynamixelController::enableTorque(uint8_t id) 
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 64, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(this->get_logger(), "Torque enable failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
    } 
    else if (dxl_error != 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Torque enable error: %s", packetHandler_->getRxPacketError(dxl_error));
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Torque enabled for motor ID %d", id);
    }
}

void DynamixelController::disableTorque(uint8_t id) 
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 64, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(this->get_logger(), "Torque disable failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
    } 
    else if (dxl_error != 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Torque disable error: %s", packetHandler_->getRxPacketError(dxl_error));
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Torque disabled for motor ID %d", id);
    }
}

void DynamixelController::setPositionControlMode(uint8_t id) 
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 11, 3, &dxl_error); // 3は位置制御モード
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set position control mode: %s", packetHandler_->getTxRxResult(dxl_comm_result));
    } 
    else if (dxl_error != 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Position control mode error: %s", packetHandler_->getRxPacketError(dxl_error));
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Position control mode set for motor ID %d", id);
    }
}

void DynamixelController::sendVelocityCommand(uint8_t id, int32_t velocity) 
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, 104, velocity, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set velocity: %s", packetHandler_->getTxRxResult(dxl_comm_result));
    } 
    else if (dxl_error != 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Velocity set error: %s", packetHandler_->getRxPacketError(dxl_error));
    }
}

void DynamixelController::sendPositionCommand(uint8_t id, int32_t position) 
{
    // 範囲を 1800 ～ 2200 に制限
    const int32_t min_position = 1948;
    const int32_t max_position = 2148;

    if (position < min_position) {
        position = min_position;  // 範囲より小さい場合は最小値にクリップ
    } else if (position > max_position) {
        position = max_position;  // 範囲より大きい場合は最大値にクリップ
    }

    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, 116, position, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set position: %s", packetHandler_->getTxRxResult(dxl_comm_result));
    } 
    else if (dxl_error != 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Position set error: %s", packetHandler_->getRxPacketError(dxl_error));
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Position set to: %d for motor ID %d", position, id);
    }
}


void DynamixelController::setLED(uint8_t id, bool state) 
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 65, state ? 1 : 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set LED: %s", packetHandler_->getTxRxResult(dxl_comm_result));
    } 
    else if (dxl_error != 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "LED set error: %s", packetHandler_->getRxPacketError(dxl_error));
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "LED %s for motor ID %d", state ? "on" : "off", id);
    }
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}
