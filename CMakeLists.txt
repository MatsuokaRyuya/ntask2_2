cmake_minimum_required(VERSION 3.8)
project(ntask2_2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# ヘッダーファイルのインクルードパスを指定
include_directories(
  include  # プロジェクトのincludeディレクトリ
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
)

# 実行可能ファイルの設定
add_executable(joy_subscriber src/joy_subscriber.cpp)
add_executable(dynamixel_controller src/dynamixel_controller.cpp)

# 必要な依存関係を追加
ament_target_dependencies(joy_subscriber rclcpp sensor_msgs std_msgs)
ament_target_dependencies(dynamixel_controller rclcpp std_msgs dynamixel_sdk)

# インストール設定
install(TARGETS
  joy_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  dynamixel_controller
  DESTINATION lib/${PROJECT_NAME})

# launchファイルのインストール
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
