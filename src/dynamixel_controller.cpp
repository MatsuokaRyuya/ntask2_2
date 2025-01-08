#include "ntask2_2/ntask2_2.hpp"

DynamixelController::DynamixelController() : Node("dynamixel_controller") 
{
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/dynamixel_motor/command", 10,
        std::bind(&DynamixelController::commandCallback, this, std::placeholders::_1));

    portHandler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
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

    motor_ids_ = {1, 2};

    for (uint8_t id : motor_ids_) {
        enableTorque(id);
    }
}

DynamixelController::~DynamixelController() 
{
    for (uint8_t id : motor_ids_) {
        disableTorque(id);
    }
    portHandler_->closePort();
}

void DynamixelController::commandCallback(const std_msgs::msg::Float64::SharedPtr msg) 
{
    double speed = msg->data;
    int32_t velocity_value = static_cast<int32_t>(speed * 10.23);
    for (uint8_t id : motor_ids_) {
        sendVelocityCommand(id, velocity_value);
    }

    RCLCPP_INFO(this->get_logger(), "Set velocity: %.2f", speed);
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


int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}

















/*
class DynamixelController : public rclcpp::Node
{
public:
    DynamixelController() : Node("dynamixel_controller") //Nodeの名前はdynamixel_controller
    {
        // トピックを購読して速度コマンドを受け取る
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/dynamixel_motor/command", 10,
            std::bind(&DynamixelController::commandCallback, this, std::placeholders::_1)); //joy_twistノードで作った/dynamixel_motor/commandトピックを受け取り、commandCallbackを呼び出す

        // Dynamixel SDKの設定
        portHandler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); // dynamixelモーターと通信するポートを設定する。ここでは/dev/ttyUSB0
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);     // 使用するdynamixelプロトコルのバージョンを設定する

        if (portHandler_->openPort()) //通信ポートを開いて正常に開いたかログに出す、開けなかったらノードをシャットダウンする
        {
            RCLCPP_INFO(this->get_logger(), "Port opened successfully.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the port.");
            rclcpp::shutdown();
        }

        if (portHandler_->setBaudRate(57600)) //dynamixelモーターとの通信速度（BaudRate）を57600にする、それが成功したかをログに出す
        {
            RCLCPP_INFO(this->get_logger(), "Baudrate set successfully.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate.");
            rclcpp::shutdown();
        }

        //制御対象のモーターIDを１にする、enableTorque関数でモーターのトルクを有効化する
        motor_id_ = 1;
        enableTorque(motor_id_);
    }

    ~DynamixelController() //ノード終了時にモーターのトルクを無効化する、通信ポートを閉じる
    {
        disableTorque(motor_id_);
        portHandler_->closePort();
    }

private: 
    void commandCallback(const std_msgs::msg::Float64::SharedPtr msg) //publicで作ったcommandCallbackを設定する
    {
        // トピックから受け取った速度コマンドを適用
        double speed = msg->data;

        // Dynamixelモーターに速度コマンドを送信
        int32_t velocity_value = static_cast<int32_t>(speed * 10.23); // 速度をモーターの内部フォーマットに変換
        sendVelocityCommand(motor_id_, velocity_value); //速度をモーターに送信する

        RCLCPP_INFO(this->get_logger(), "Set velocity: %.2f", speed);
    }

    void enableTorque(uint8_t id) //publicで作ったenableTorqueを設定する、dynamixelの制御テーブルでアドレス64に１を書き込む
    {
        // トルクを有効化
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

    void disableTorque(uint8_t id) //disableTorqueを設定する、こっちではアドレス64に０を書き込む
    {
        // トルクを無効化
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

    void sendVelocityCommand(uint8_t id, int32_t velocity) //sendvelocityCommandを設定する、アドレス104に値を書き込む
    {
        // 速度コマンドを送信
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

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_; // トピック購読
    dynamixel::PortHandler *portHandler_;  // Dynamixelポートハンドラ
    dynamixel::PacketHandler *packetHandler_; // Dynamixelパケットハンドラ
    uint8_t motor_id_; // 使用するモーターのID
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelController>());
    rclcpp::shutdown();
    return 0;
}

*/
/*

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"  // Dynamixel SDK

// Dynamixelの設定
#define DEVICENAME              "/dev/ttyUSB0"   // USBシリアルポートの名前（適宜変更）
#define BAUDRATE                57600            // 通信速度
#define MOTOR_ID                1                // モーターID（DynamixelのID）
#define PROTOCOL_VERSION        2.0              // プロトコルバージョン
#define MOTOR_MIN_SPEED         0                // 最小速度
#define MOTOR_MAX_SPEED         1023             // 最大速度

class JoyToDynamixel : public rclcpp::Node {　　　ジョイパッドの入力をモーターの速度に変換するノード
public:
    JoyToDynamixel() : Node("joy_to_dynamixel") {
        // ジョイパッドのトピックを購読
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToDynamixel::joyCallback, this, std::placeholders::_1));

        // Dynamixelモーター制御の初期化
        port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!port_handler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port!");
        }

        if (!port_handler_->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate!");
        }

        RCLCPP_INFO(this->get_logger(), "JoyToDynamixel node has started.");
    }

    ~JoyToDynamixel() {
        // Dynamixel通信の終了
        port_handler_->closePort();
    }

private:
    // ジョイパッドからの入力を受け取るコールバック関数
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // ジョイスティックの軸情報（axes[1]は上下スティック）
        double speed = msg->axes[1] * MOTOR_MAX_SPEED;  // スティックの上下の入力を速度に変換

        // モーターの速度範囲を制限
        if (speed < MOTOR_MIN_SPEED) speed = MOTOR_MIN_SPEED;
        if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;

        // 速度コマンドをDynamixelモーターに送信
        int result = packet_handler_->write2ByteTxRx(
            port_handler_, MOTOR_ID, 32, static_cast<uint16_t>(speed));  // 32番目の制御レジスタ（速度制御）に書き込む

        if (result != dynamixel::COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set motor speed!");
        }

        RCLCPP_INFO(this->get_logger(), "Published speed: %.2f", speed);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    dynamixel::PortHandler *port_handler_;
    dynamixel::PacketHandler *packet_handler_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyToDynamixel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

*/