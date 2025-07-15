#include "md_motor_driver/com.hpp"
#include "md_motor_driver/main.hpp"
#include "md_motor_driver/global.hpp"

#include "md_motor_interface_msgs/msg/md_robot_msg1.hpp"
#include "md_motor_interface_msgs/msg/md_robot_msg2.hpp"

#include <cstdint>

void robot_test_log() {
  RCLCPP_INFO(rclcpp::get_logger("robot"), "✅ robot.cpp compiled and linked");
}