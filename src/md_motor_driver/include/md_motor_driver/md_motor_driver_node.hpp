// md_motor_driver_node.hpp
#pragma once

#define ENABLE_MD_MESSAGE          
#define VELOCITY_CONSTANT_VALUE 9.5492743
#define constrain(amt, low, high) ((amt) <= (low) ? (low) : ((amt) >= (high) ? (high) : (amt)))

#define LEFT  0
#define RIGHT 1

#define MD_PROTOCOL_POS_PID             3
#define MD_PROTOCOL_POS_DATA_LEN        4
#define MD_PROTOCOL_POS_DATA_START      5

#define ENABLE_SERIAL_DEBUG             0

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include "md_motor_interface_msgs/msg/md_robot_msg1.hpp"
#include "md_motor_interface_msgs/msg/md_robot_msg2.hpp"

#include "md_motor_driver/SerialComm.hpp"
#include "md_motor_driver/com.hpp"

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/empty.hpp>

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <cfloat>
#include <cmath>

using namespace std;

class MdMotorDriverNode : public rclcpp::Node, public std::enable_shared_from_this<MdMotorDriverNode>
{
public:
    MdMotorDriverNode();
    void run();

private:
    // ROS 통신 객체
    md_motor_interface_msgs::msg::MdRobotMsg1 md_robot_msg_pid_pnt_main_data;
    rclcpp::Publisher<md_motor_interface_msgs::msg::MdRobotMsg1>::SharedPtr pub_msg1_;
    
    md_motor_interface_msgs::msg::MdRobotMsg2 md_robot_msg_pid_robot_monitor;
    rclcpp::Publisher<md_motor_interface_msgs::msg::MdRobotMsg2>::SharedPtr pub_msg2_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reset_pos_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reset_alarm_;

    rclcpp::TimerBase::SharedPtr vel_cmd_rcv_timeout_timer;

    // 로봇 파라미터 및 내부 상태
    ROBOT_PARAMETER_t robotParamData;

    PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
    PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
    PID_IO_MONITOR_t curr_pid_io_monitor;
    PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;

    INIT_SETTING_STATE_t fgInitsetting;
    SETTINNG_PARAM_STEP_t byCntInitStep;
    
    uint16_t byCntComStep;
    uint32_t velCmdUpdateCount;
    uint32_t velCmdRcvCount;
    uint32_t old_velCmdRcvCount;
    uint32_t old_pid_response_receive_count;
    uint32_t pid_response_receive_count;
    uint32_t pid_request_cmd_vel_count;
    uint16_t check_connection_retry_count;

    bool old_mdui_mdt_connection_state;
    bool old_remote_pc_connection_state;
    bool mdui_mdt_connection_state;
    bool remote_pc_connection_state;
    bool reset_pos_flag;
    bool reset_alarm_flag;

    double goal_cmd_speed;
    double goal_cmd_ang_speed;

    // AnalyzeReceiveData
    uint32_t byPacketNum;
    uint32_t rcv_step;
    uint8_t byChkSum;
    uint16_t byMaxDataNum;
    uint16_t byDataNum;

    // ReceiveSerialData
    uint8_t tempBuffer[250];
    uint8_t tempLength;

    // PutMdData
    uint8_t serial_comm_rcv_buff[MAX_PACKET_SIZE];
    uint8_t serial_comm_snd_buff[MAX_PACKET_SIZE];

    // 시리얼 통신 객체
    std::unique_ptr<SerialComm> ser;

    // 콜백 함수
    void velCmdRcvTimeoutCallback();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void resetPositionCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void resetAlarmCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // 내부 처리 함수
    void loadParameters();
    void ReceiveSerialData();
    int AnalyzeReceivedData(uint8_t byArray[], uint8_t byBufNum);
    int MdReceiveProc();
    int PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length);
    void InitMotorParameter();
    void RequestRobotStatus();
    uint8_t CalCheckSum(uint8_t *pData, uint16_t length);

    int16_t* RobotSpeedToRPMSpeed(double linear, double angular);
    void MakeMDRobotMessage1(const PID_PNT_MAIN_DATA_t *pData);
    void MakeMDRobotMessage2(PID_ROBOT_MONITOR_t *pData);
    void PubRobotRPMMessage();
    void PubRobotOdomMessage();
};