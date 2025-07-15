#include "md_motor_driver/com.hpp"
#include "md_motor_driver/main.hpp"
#include "md_motor_driver/global.hpp"

#include "md_motor_interface_msgs/msg/md_robot_msg1.hpp"
#include "md_motor_interface_msgs/msg/md_robot_msg2.hpp"
#include "md_motor_interface_msgs/msg/pose.hpp"
//Message 패키지 만들때는 대문자 시작, 카멜케이스 하라더니만, 
//idl이 만들어주는건 다 소문자에 스네이크 케이스로 만들어주노

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <queue>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Hello from main!");

  com_test_log();
  robot_test_log();

  rclcpp::shutdown();
  return 0;
}