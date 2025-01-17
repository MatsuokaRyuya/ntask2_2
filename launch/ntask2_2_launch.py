from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # JoySubscriber ノードを起動
        Node(
            package='ntask2_2',
            executable='joy_subscriber',
            name='joy_subscriber',
            output='screen'
        ),
        # DynamixelController ノードを起動
        Node(
            package='ntask2_2',
            executable='dynamixel_controller',
            name='dynamixel_controller',
            output='screen',
            parameters=[
                {'port_name': '/dev/ttyUSB0'},  # シリアルポートの名前
                {'baud_rate': 57600}           # 通信速度
            ]
        ),
        Node(
            package='joy',
            executable='joy_node',  # publisher.cpp で作成した実行可能ファイル
            name='joy_node',
            output='screen',
        ),
    ])
