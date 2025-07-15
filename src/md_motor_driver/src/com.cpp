#include "md_motor_driver/com.hpp"
#include "md_motor_driver/main.hpp"
#include "md_motor_driver/global.hpp"


void com_test_log() {
  RCLCPP_INFO(rclcpp::get_logger("com"), "✅ com.cpp compiled and linked");
}