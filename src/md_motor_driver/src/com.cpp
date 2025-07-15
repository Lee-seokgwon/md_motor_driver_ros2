#include "md_motor_driver/com.hpp"
#include "md_motor_driver/main.hpp"
#include "md_motor_driver/global.hpp"

PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
PID_IO_MONITOR_t curr_pid_io_monitor;

void com_test_log() {
  RCLCPP_INFO(rclcpp::get_logger("com"), "✅ com.cpp compiled and linked");
}