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

SETTINNG_PARAM_STEP_t byCntInitStep;
uint16_t byCntComStep;
uint32_t velCmdUpdateCount;
uint32_t velCmdRcvCount;
uint32_t pid_response_receive_count;
uint32_t pid_request_cmd_vel_count;
volatile bool mdui_mdt_connection_state;
volatile bool remote_pc_connection_state;

INIT_SETTING_STATE_t fgInitsetting;
uint16_t check_connection_retry_count;

double goal_cmd_speed;             // m/sec
double goal_cmd_ang_speed;         // radian/sec
bool reset_pos_flag;
bool reset_alarm_flag;

extern PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
extern PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;

extern int InitSerialComm(void);
extern int16_t * RobotSpeedToRPMSpeed(double linear, double angular);

std::string serial_port;


void PubRobotRPMMessage(void)               // This is the message used by default
{
    return;
}

void PubRobotOdomMessage(void)             // Use only when using MDUI
{
    return;
}








// test logging code
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Hello from main!");

  com_test_log();
  robot_test_log();

  rclcpp::shutdown();
  return 0;
}