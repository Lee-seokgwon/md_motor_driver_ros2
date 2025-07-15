#include "md_motor_driver/com.hpp"
#include "md_motor_driver/main.hpp"
#include "md_motor_driver/global.hpp"

#include "md_motor_interface_msgs/msg/md_robot_msg1.hpp"
#include "md_motor_interface_msgs/msg/md_robot_msg2.hpp"
#include "md_motor_interface_msgs/msg/pose.hpp"
//    .hpp 헤더 경로는 소문자 + 스네이크케이스
//   내부 타입은 원래 .msg 파일 이름과 동일한 CamelCase
//이는 rosidl 빌드시 자동 생성되는 규칙이니 그대로 따라가면 됨.

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <queue>

using md_motor_interface_msgs::msg::MdRobotMsg1;
using md_motor_interface_msgs::msg::MdRobotMsg2;

MdRobotMsg1 md_robot_msg_pid_pnt_main_data;
MdRobotMsg2 md_robot_msg_pid_robot_monitor;

ROBOT_PARAMETER_t robotParamData;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Hello from main!");

  com_test_log();
  robot_test_log();

  rclcpp::shutdown();
  return 0;
}