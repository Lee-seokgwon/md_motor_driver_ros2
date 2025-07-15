#include "md_motor_driver/com.hpp"
#include "md_motor_driver/main.hpp"
#include "md_motor_driver/global.hpp"

#include "md_motor_interface_msgs/msg/md_robot_msg1.hpp"
#include "md_motor_interface_msgs/msg/md_robot_msg2.hpp"

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#define ENABLE_MD_MESSAGE          

#define VELOCITY_CONSTANT_VALUE 9.5492743
#define constrain(amt, low, high) ((amt) <= (low) ? (low) : ((amt) >= (high) ? (high) : (amt)))

#define LEFT  0
#define RIGHT 1

using md_motor_interface_msgs::msg::MdRobotMsg1;
using md_motor_interface_msgs::msg::MdRobotMsg2;

extern MdRobotMsg1 md_robot_msg_pid_pnt_main_data;
extern MdRobotMsg2 md_robot_msg_pid_robot_monitor;
extern PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
extern PID_IO_MONITOR_t curr_pid_io_monitor;

static rclcpp::Logger logger = rclcpp::get_logger("robot");

//목표 선속도 각속도를 받아서 , 로봇의 좌우측 바퀴의 rpm을 계산하여 array로 넘겨주는 함수
int16_t* RobotSpeedToRPMSpeed(double linear, double angular)
{
    double wheel_radius = robotParamData.wheel_radius;
    double wheel_separation = robotParamData.nWheelLength;
    double reduction = static_cast<double>(robotParamData.nGearRatio);
    double wheel_velocity_cmd[2];
    static int16_t goal_rpm_speed[2];

    wheel_velocity_cmd[LEFT]  = linear - (angular * wheel_separation / 2);
    wheel_velocity_cmd[RIGHT] = linear + (angular * wheel_separation / 2);

    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -robotParamData.nMaxRPM, robotParamData.nMaxRPM);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -robotParamData.nMaxRPM, robotParamData.nMaxRPM);

    goal_rpm_speed[0] = static_cast<int16_t>(wheel_velocity_cmd[LEFT]);
    goal_rpm_speed[1] = static_cast<int16_t>(wheel_velocity_cmd[RIGHT]);

    return goal_rpm_speed;
}

//모터상태를 담은 구조체 포인터를 받아서, ROS message로 만들어서 Publish 하기위한 데이터 포매팅 함수
//md_robot_msg_pid_pnt_main_data 는 MdRobotMsg1 (커스텀 msg 인터페이스) 의 인스턴스
void MakeMDRobotMessage1(PID_PNT_MAIN_DATA_t *pData)
{
    static bool first_cal = false;
    static rclcpp::Time previous_time = rclcpp::Clock().now();
    rclcpp::Time curr_time = rclcpp::Clock().now();

    double interval_time = (curr_time - previous_time).seconds();
    previous_time = curr_time;

    md_robot_msg_pid_pnt_main_data.interval_time = interval_time;
    md_robot_msg_pid_pnt_main_data.motor1_pos = pData->mtr_pos_id1;
    md_robot_msg_pid_pnt_main_data.motor2_pos = pData->mtr_pos_id2;
    md_robot_msg_pid_pnt_main_data.motor1_rpm = pData->rpm_id1;
    md_robot_msg_pid_pnt_main_data.motor2_rpm = pData->rpm_id2;
    md_robot_msg_pid_pnt_main_data.motor1_state = pData->mtr_state_id1.val;
    md_robot_msg_pid_pnt_main_data.motor2_state = pData->mtr_state_id2.val;
    md_robot_msg_pid_pnt_main_data.input_voltage = static_cast<float>(curr_pid_io_monitor.input_voltage / 10.0);

#ifdef ENABLE_MD_MESSAGE
    RCLCPP_INFO(logger, "interval time1: %f, pos1: %d, pos2: %d, rpm1: %d rpm2: %d, input voltage: %f",
        interval_time,
        md_robot_msg_pid_pnt_main_data.motor1_pos,
        md_robot_msg_pid_pnt_main_data.motor2_pos,
        md_robot_msg_pid_pnt_main_data.motor1_rpm,
        md_robot_msg_pid_pnt_main_data.motor2_rpm,
        md_robot_msg_pid_pnt_main_data.input_voltage);
#endif
}

//모터상태를 담은 구조체 포인터를 받아서, ROS message로 만들어서 Publish 하기위한 데이터 포매팅 함수
//MDUI가 있는 경우에 사용됨
void MakeMDRobotMessage2(PID_ROBOT_MONITOR_t *pData)
{
    static bool first_cal = false;
    static rclcpp::Time previous_time = rclcpp::Clock().now();
    rclcpp::Time curr_time = rclcpp::Clock().now();

    double interval_time = (curr_time - previous_time).seconds();
    previous_time = curr_time;

    md_robot_msg_pid_robot_monitor.interval_time = interval_time;
    md_robot_msg_pid_robot_monitor.x_pos = pData->lTempPosi_x;
    md_robot_msg_pid_robot_monitor.y_pos = pData->lTempPosi_y;
    md_robot_msg_pid_robot_monitor.angule = pData->sTempTheta;

    if (robotParamData.reverse_direction == 0) {
        md_robot_msg_pid_robot_monitor.us_1 = pData->byUS1;
        md_robot_msg_pid_robot_monitor.us_2 = pData->byUS2;
    } else {
        md_robot_msg_pid_robot_monitor.us_1 = pData->byUS2;
        md_robot_msg_pid_robot_monitor.us_2 = pData->byUS1;
    }

    md_robot_msg_pid_robot_monitor.platform_state = pData->byPlatStatus.val;
    md_robot_msg_pid_robot_monitor.linear_velocity = pData->linear_velocity;
    md_robot_msg_pid_robot_monitor.angular_velocity = pData->angular_velocity;
    md_robot_msg_pid_robot_monitor.input_voltage = static_cast<float>(curr_pid_robot_monitor2.sVoltIn / 10.0);

#ifdef ENABLE_MD_MESSAGE
    RCLCPP_INFO(logger, "interval time2: %f, input_voltage: %f", interval_time, md_robot_msg_pid_robot_monitor.input_voltage);
#endif    
}


//for test logging
void robot_test_log() {
  RCLCPP_INFO(rclcpp::get_logger("robot"), "✅ robot.cpp compiled and linked");
}