#include "md_motor_driver/md_motor_driver_node.hpp"

#include <chrono>
using namespace std::chrono_literals;

MdMotorDriverNode::MdMotorDriverNode()
: Node("md_motor_driver_node"),
  fgInitsetting(INIT_SETTING_STATE_NONE),
  byCntInitStep(SETTING_PARAM_STEP_PID_PNT_VEL_CMD),
  byCntComStep(0),
  velCmdUpdateCount(0),
  velCmdRcvCount(0),
  old_velCmdRcvCount(0),
  old_pid_response_receive_count(0),
  pid_response_receive_count(0),
  pid_request_cmd_vel_count(0),
  check_connection_retry_count(0),
  old_mdui_mdt_connection_state(false),
  old_remote_pc_connection_state(false),
  mdui_mdt_connection_state(false),
  remote_pc_connection_state(false),
  reset_pos_flag(false),
  reset_alarm_flag(false),
  goal_cmd_speed(0.0),
  goal_cmd_ang_speed(0.0),
  byPacketNum(0),
  rcv_step(0),
  byChkSum(0),
  byMaxDataNum(0),
  byDataNum(0),
  tempBuffer{0},
  tempLength(0),
  serial_comm_rcv_buff{0},
  serial_comm_snd_buff{0}
{   
    auto names = this->list_parameters({}, 10);
    RCLCPP_INFO(this->get_logger(), "===== PARAMETER NAMES =====");
    for (auto& name : names.names) {
        RCLCPP_INFO(this->get_logger(), "param: %s", name.c_str());
    }
    // Declare and load parameters
    loadParameters();

    // Create publisher
    pub_msg1_ = this->create_publisher<md_motor_interface_msgs::msg::MdRobotMsg1>("md_robot_message1", 10);
    pub_msg2_ = this->create_publisher<md_motor_interface_msgs::msg::MdRobotMsg2>("md_robot_message2", 10);

    // Create subscriber
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&MdMotorDriverNode::cmdVelCallback, this, std::placeholders::_1)
    );

    sub_reset_pos_ = this->create_subscription<std_msgs::msg::Bool>(
        "reset_position", 10,
        std::bind(&MdMotorDriverNode::resetPositionCallback, this, std::placeholders::_1)
    );

    sub_reset_alarm_ = this->create_subscription<std_msgs::msg::Bool>(
        "reset_alarm", 10,
        std::bind(&MdMotorDriverNode::resetAlarmCallback, this, std::placeholders::_1)
    );

    // Create timer (1Hz)
    vel_cmd_rcv_timeout_timer = this->create_wall_timer(
        1s, std::bind(&MdMotorDriverNode::velCmdRcvTimeoutCallback, this)
    );

    // Init serial communication
    ser = std::make_unique<SerialComm>();
    if (!ser->open(robotParamData.serial_port, robotParamData.nBaudrate)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "[Init done]");
}

void MdMotorDriverNode::PubRobotRPMMessage()
{
    pub_msg1_->publish(md_robot_msg_pid_pnt_main_data);
}

void MdMotorDriverNode::PubRobotOdomMessage()
{
    pub_msg2_->publish(md_robot_msg_pid_robot_monitor);
}


void MdMotorDriverNode::MakeMDRobotMessage1(const PID_PNT_MAIN_DATA_t* pData)
{
    static bool first_cal = false;
    static rclcpp::Time previous_time = this->now();  // ROS2 노드의 현재 시간
    rclcpp::Time curr_time = this->now();

    double interval_time = (curr_time - previous_time).seconds();
    previous_time = curr_time;

    md_robot_msg_pid_pnt_main_data.interval_time = interval_time;
    md_robot_msg_pid_pnt_main_data.motor1_pos = pData->mtr_pos_id1;
    md_robot_msg_pid_pnt_main_data.motor2_pos = pData->mtr_pos_id2;
    md_robot_msg_pid_pnt_main_data.motor1_rpm = pData->rpm_id1;
    md_robot_msg_pid_pnt_main_data.motor2_rpm = pData->rpm_id2;
    md_robot_msg_pid_pnt_main_data.motor1_state = pData->mtr_state_id1.val;
    md_robot_msg_pid_pnt_main_data.motor2_state = pData->mtr_state_id2.val;
    md_robot_msg_pid_pnt_main_data.input_voltage =
        static_cast<float>(curr_pid_io_monitor.input_voltage / 10.0);

#ifdef ENABLE_MD_MESSAGE
    RCLCPP_INFO(this->get_logger(),
        "interval time1: %f, pos1: %d, pos2: %d, rpm1: %d, rpm2: %d, input voltage: %f",
        interval_time,
        md_robot_msg_pid_pnt_main_data.motor1_pos,
        md_robot_msg_pid_pnt_main_data.motor2_pos,
        md_robot_msg_pid_pnt_main_data.motor1_rpm,
        md_robot_msg_pid_pnt_main_data.motor2_rpm,
        md_robot_msg_pid_pnt_main_data.input_voltage);
#endif
}

void MdMotorDriverNode::MakeMDRobotMessage2(PID_ROBOT_MONITOR_t *pData)
{
    static bool first_cal = false;
    static rclcpp::Time previous_time = this->now();
    rclcpp::Time curr_time = this->now();

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
    RCLCPP_INFO(this->get_logger(), "interval time2: %f, input_voltage: %f", 
                interval_time, md_robot_msg_pid_robot_monitor.input_voltage);
#endif    
}


uint8_t MdMotorDriverNode::CalCheckSum(uint8_t *pData, uint16_t length)
{
    uint16_t sum;
    sum = 0;

    for(int i = 0; i < length; i++) {
        sum += *pData++;
    }

    sum = ~sum + 1; //check sum
    return sum;
}

int MdMotorDriverNode::PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length)
{
    uint8_t *p;
    uint16_t len = 0;

    serial_comm_snd_buff[len++] = rmid;
    serial_comm_snd_buff[len++] = robotParamData.nIDPC;
    serial_comm_snd_buff[len++] = 1;
    serial_comm_snd_buff[len++] = static_cast<uint8_t>(pid);
    serial_comm_snd_buff[len++] = length;

    // 데이터 복사
    p = &serial_comm_snd_buff[len];
    memcpy(p, pData, length);
    len += length;

    // 체크섬 추가
    serial_comm_snd_buff[len++] = CalCheckSum(serial_comm_snd_buff, len);

    // 포트 열림 여부 확인 후 전송
    if (ser->isOpen()) {
        ser->write(serial_comm_snd_buff, len);
    }

    return 1;
}

void MdMotorDriverNode::loadParameters()
{   
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baudrate", 57600);
    this->declare_parameter<int>("reverse_direction", 0);
    this->declare_parameter<int>("maxrpm", 1000);
    this->declare_parameter<int>("enable_encoder", 0);
    this->declare_parameter<int>("slow_start", 300);
    this->declare_parameter<int>("slow_down", 300);
    this->declare_parameter<double>("wheel_length", 0.454);
    this->declare_parameter<int>("use_MDUI", 0);
    this->declare_parameter<int>("reduction", 30);
    this->declare_parameter<double>("wheel_radius", 0.0935);
    this->declare_parameter<int>("encoder_PPR", 900);
    // Load into robotParamData
    this->get_parameter("serial_port", robotParamData.serial_port);
    this->get_parameter("serial_baudrate", robotParamData.nBaudrate);
    this->get_parameter("reverse_direction", robotParamData.reverse_direction);
    this->get_parameter("maxrpm", robotParamData.nMaxRPM);
    this->get_parameter("enable_encoder", robotParamData.enable_encoder);
    this->get_parameter("slow_start", robotParamData.nSlowstart);
    this->get_parameter("slow_down", robotParamData.nSlowdown);
    this->get_parameter("wheel_length", robotParamData.nWheelLength);
    this->get_parameter("use_MDUI", robotParamData.use_MDUI);
    this->get_parameter("reduction", robotParamData.nGearRatio);
    this->get_parameter("wheel_radius", robotParamData.wheel_radius);
    this->get_parameter("encoder_PPR", robotParamData.encoder_PPR);

    // Derived value
    robotParamData.nDiameter = static_cast<int>(robotParamData.wheel_radius * 2.0 * 1000.0);

    // Debug print
    RCLCPP_INFO(this->get_logger(), "[Parameters loaded]");
    RCLCPP_INFO(this->get_logger(), "Serial port : %s", robotParamData.serial_port.c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate    : %d", robotParamData.nBaudrate);
    RCLCPP_INFO(this->get_logger(), "Wheel radius(m): %.3f", robotParamData.wheel_radius);
    RCLCPP_INFO(this->get_logger(), "Diameter(mm)   : %d", robotParamData.nDiameter);
}

void MdMotorDriverNode::InitMotorParameter()
{
    switch (byCntInitStep) //byCntInitStep <- run 함수 내에서 초기화됨.
    {
        case SETTING_PARAM_STEP_PID_PNT_VEL_CMD:
        {
            PID_PNT_VEL_CMD_t cmd_data{};
            cmd_data.enable_id1 = 1;
            cmd_data.rpm_id1 = 0;
            cmd_data.enable_id2 = 1;
            cmd_data.rpm_id2 = 0;

            if (robotParamData.use_MDUI == 1)
                cmd_data.req_monitor_id = REQUEST_PID_ROBOT_MONITOR;
            else
                cmd_data.req_monitor_id = REQUEST_PNT_MAIN_DATA;

            pid_response_receive_count = 0;
            pid_request_cmd_vel_count = 1;

            PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, reinterpret_cast<uint8_t*>(&cmd_data), sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_WAIT;
            break;
        }

        case SETTING_PARAM_WAIT:
        {
            if (pid_response_receive_count > 0) {
                pid_response_receive_count = 0;
                byCntInitStep = SETTING_PARAM_STEP_PID_ROBOT_PARAM;
            } else {
                check_connection_retry_count++;
                if (check_connection_retry_count >= MAX_CONNECTION_CHECK_COUNT) {
                    RCLCPP_ERROR(this->get_logger(), "!!! Error RS232(MDUI) or RS485(MDT) !!!");
                    fgInitsetting = INIT_SETTING_STATE_ERROR;
                    byCntInitStep = SETTING_PARAM_STEP_DONE;
                } else {
                    byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
                }
            }
            break;
        }

        case SETTING_PARAM_STEP_PID_ROBOT_PARAM:
        {
            if (robotParamData.use_MDUI == 1) {
                PID_ROBOT_PARAM_t cmd_data{};
                cmd_data.nDiameter = static_cast<uint16_t>(robotParamData.nDiameter);
                cmd_data.nWheelLength = static_cast<uint16_t>(robotParamData.nWheelLength * 1000.0);
                cmd_data.nGearRatio = static_cast<uint16_t>(robotParamData.nGearRatio);

                PutMdData(PID_ROBOT_PARAM, MID_MDUI, reinterpret_cast<uint8_t*>(&cmd_data), sizeof(cmd_data));
            }
            byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
            break;
        }

        case SETTING_PARAM_STEP_PID_POSI_RESET:
        {
            uint8_t dummy = 0;
            PutMdData(PID_POSI_RESET, robotParamData.nRMID, &dummy, 1);
            byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_START;
            break;
        }

        case SETTING_PARAM_STEP_PID_SLOW_START:
        {
            PID_SLOW_START_t cmd_data{};
            cmd_data.value = robotParamData.nSlowstart;
            PutMdData(PID_SLOW_START, robotParamData.nRMID, reinterpret_cast<uint8_t*>(&cmd_data), sizeof(cmd_data));
            byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_DOWN;
            break;
        }

        case SETTING_PARAM_STEP_PID_SLOW_DOWN:
        {
            PID_SLOW_DOWN_t cmd_data{};
            cmd_data.value = robotParamData.nSlowdown;
            PutMdData(PID_SLOW_DOWN, robotParamData.nRMID, reinterpret_cast<uint8_t*>(&cmd_data), sizeof(cmd_data));
            byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD;
            break;
        }

        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD:
        {
            uint8_t cmd_data = (robotParamData.reverse_direction == 0) ? 1 : 0;
            PutMdData(PID_INV_SIGN_CMD, robotParamData.nRMID, &cmd_data, 1);
            byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD2;
            break;
        }

        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD2:
        {
            uint8_t cmd_data = (robotParamData.reverse_direction == 0) ? 0 : 1;
            PutMdData(PID_INV_SIGN_CMD2, robotParamData.nRMID, &cmd_data, 1);
            byCntInitStep = SETTING_PARAM_STEP_PID_USE_EPOSI;
            break;
        }

        case SETTING_PARAM_STEP_PID_USE_EPOSI:
        {
            uint8_t cmd_data = (robotParamData.enable_encoder == 0) ? 0 : 1;
            PutMdData(PID_USE_POSI, robotParamData.nRMID, &cmd_data, 1);
            byCntInitStep = SETTING_PARAM_STEP_PID_PPR;
            break;
        }

        case SETTING_PARAM_STEP_PID_PPR:
        {
            if (robotParamData.use_MDUI == 1 && robotParamData.enable_encoder == 1) {
                PID_PPR_t cmd_data{};
                cmd_data.PPR = robotParamData.encoder_PPR;
                PutMdData(PID_PPR, robotParamData.nRMID, reinterpret_cast<uint8_t*>(&cmd_data), sizeof(cmd_data));
            }

            byCntInitStep = SETTING_PARAM_STEP_DONE;

            if (pid_request_cmd_vel_count == 2)
                fgInitsetting = INIT_SETTING_STATE_OK;
            else {
                fgInitsetting = INIT_SETTING_STATE_ERROR;
                RCLCPP_ERROR(this->get_logger(), robotParamData.use_MDUI == 1 ?
                    "!!! Error RS232 interface !!!" : "!!! Error RS485 interface !!!");
            }
            break;
        }

        default:
            break;
    }

}


void MdMotorDriverNode::RequestRobotStatus()
{
    int16_t* pGoalRPMSpeed = nullptr;

    switch (byCntComStep)
    {
        case 0:
        {
            if (velCmdUpdateCount > 0)
            {
                velCmdUpdateCount = 0;

                PID_PNT_VEL_CMD_t pid_pnt_vel_cmd{};
                if (mdui_mdt_connection_state)
                {
                    pGoalRPMSpeed = RobotSpeedToRPMSpeed(goal_cmd_speed, goal_cmd_ang_speed);
                }
                else
                {
                    static int16_t zero_rpm[2] = {0, 0};
                    pGoalRPMSpeed = zero_rpm;
                }

                RCLCPP_INFO(this->get_logger(), "Goal %.2f, %.2f, RPM L:%d, R:%d",
                            goal_cmd_speed, goal_cmd_ang_speed, pGoalRPMSpeed[0], pGoalRPMSpeed[1]);

                pid_pnt_vel_cmd.enable_id1 = 1;
                pid_pnt_vel_cmd.rpm_id1 = pGoalRPMSpeed[0];
                pid_pnt_vel_cmd.enable_id2 = 1;
                pid_pnt_vel_cmd.rpm_id2 = pGoalRPMSpeed[1];
                pid_pnt_vel_cmd.req_monitor_id = (robotParamData.use_MDUI == 1) ?
                    REQUEST_PID_ROBOT_MONITOR : REQUEST_PNT_MAIN_DATA;

                PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID,
                          reinterpret_cast<const uint8_t*>(&pid_pnt_vel_cmd), sizeof(pid_pnt_vel_cmd));

                pid_request_cmd_vel_count++;
            }

            if (robotParamData.use_MDUI == 1 && curr_pid_robot_monitor.byPlatStatus.bits.bEmerSW == 1)
            {
                byCntComStep = 3;
                break;
            }

            if (reset_pos_flag || reset_alarm_flag)
                byCntComStep = 3;
            else
                byCntComStep = 4;

            break;
        }

        case 3:
        {
            if (robotParamData.use_MDUI == 1 && curr_pid_robot_monitor.byPlatStatus.bits.bEmerSW == 1)
            {
                PID_PNT_TQ_OFF_t pid_pnt_tq_off{};
                pid_pnt_tq_off.enable_id1 = 1;
                pid_pnt_tq_off.enable_id2 = 1;
                pid_pnt_tq_off.req_monitor_id = REQUEST_PNT_MAIN_DATA;
                PutMdData(PID_PNT_TQ_OFF, robotParamData.nRMID,
                          reinterpret_cast<const uint8_t*>(&pid_pnt_tq_off), sizeof(pid_pnt_tq_off));
            }

            if (reset_pos_flag)
            {
                reset_pos_flag = false;
                uint8_t dummy = 0;
                PutMdData(PID_POSI_RESET, robotParamData.nRMID, &dummy, 1);
            }
            else if (reset_alarm_flag)
            {
                reset_alarm_flag = false;
                uint8_t cmd_pid = CMD_ALARM_RESET;
                PutMdData(PID_COMMAND, robotParamData.nRMID, &cmd_pid, 1);
            }

            byCntComStep = 0;
            break;
        }

        case 4:
        {
            uint8_t request_pid = (robotParamData.use_MDUI == 0) ?
                PID_IO_MONITOR : PID_ROBOT_MONITOR2;

            RCLCPP_INFO(this->get_logger(), "REQ: %s",
                        (request_pid == PID_IO_MONITOR) ? "PID_IO_MONITOR" : "PID_ROBOT_MONITOR2");

            PutMdData(PID_REQ_PID_DATA, robotParamData.nRMID,
                      &request_pid, 1);

            byCntComStep = 0;
            break;
        }

        default:
            byCntComStep = 0;
            break;
    }
}

int16_t* MdMotorDriverNode::RobotSpeedToRPMSpeed(double linear, double angular)
{
    static int16_t goal_rpm_speed[2];

    double wheel_radius = robotParamData.wheel_radius;
    double wheel_separation = robotParamData.nWheelLength;
    double reduction = static_cast<double>(robotParamData.nGearRatio);

    double wheel_velocity_cmd[2];

    // 차동 구동 속도 변환
    wheel_velocity_cmd[LEFT]  = linear - (angular * wheel_separation / 2.0);
    wheel_velocity_cmd[RIGHT] = linear + (angular * wheel_separation / 2.0);

    // 선속도 → RPM 변환 (VELOCITY_CONSTANT_VALUE는 M_PI * 2 / 60.0 등으로 정의돼 있어야 함)
    wheel_velocity_cmd[LEFT]  = std::clamp(
        wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction,
        -static_cast<double>(robotParamData.nMaxRPM),
        static_cast<double>(robotParamData.nMaxRPM)
    );

    wheel_velocity_cmd[RIGHT] = std::clamp(
        wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction,
        -static_cast<double>(robotParamData.nMaxRPM),
        static_cast<double>(robotParamData.nMaxRPM)
    );

    goal_rpm_speed[0] = static_cast<int16_t>(wheel_velocity_cmd[LEFT]);
    goal_rpm_speed[1] = static_cast<int16_t>(wheel_velocity_cmd[RIGHT]);

    return goal_rpm_speed;
}

int MdMotorDriverNode::MdReceiveProc(void)
{
    uint8_t* pRcvBuf = serial_comm_rcv_buff;
    uint8_t* pRcvData = &pRcvBuf[MD_PROTOCOL_POS_DATA_START];
    uint8_t byRcvPID = pRcvBuf[MD_PROTOCOL_POS_PID];
    uint8_t byRcvDataSize = pRcvBuf[MD_PROTOCOL_POS_DATA_LEN];

    switch (byRcvPID)
    {
        case PID_IO_MONITOR:
        {
            RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "RCV: PID_IO_MONITOR: %d, %ld", byRcvDataSize, sizeof(PID_IO_MONITOR_t));
            memcpy(&curr_pid_io_monitor, pRcvData, sizeof(PID_PNT_MAIN_DATA_t)); //what??? struct mismatch ... i think
            RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "Input voltage: %d", curr_pid_io_monitor.input_voltage);
            break;
        }

        case PID_ROBOT_MONITOR2:
        {
            RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "RCV: PID_ROBOT_MONITOR2: %d, %ld", byRcvDataSize, sizeof(PID_ROBOT_MONITOR2_t));
            memcpy(&curr_pid_robot_monitor2, pRcvData, sizeof(PID_ROBOT_MONITOR2_t));
            RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "Input voltage: %d", curr_pid_robot_monitor2.sVoltIn);
            break;
        }

        case PID_PNT_MAIN_DATA:
        {
            if (byRcvDataSize == sizeof(PID_PNT_MAIN_DATA_t)) {
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "RCV: PID_PNT_MAIN_DATA: %d, %ld", byRcvDataSize, sizeof(PID_PNT_MAIN_DATA_t));

                pid_response_receive_count++;
                pid_request_cmd_vel_count = 2;

                memcpy(&curr_pid_pnt_main_data, pRcvData, sizeof(PID_PNT_MAIN_DATA_t));

                MakeMDRobotMessage1(&curr_pid_pnt_main_data);
                PubRobotRPMMessage();
            }
            break;
        }

        case PID_ROBOT_MONITOR:
        {
            if (byRcvDataSize == sizeof(PID_ROBOT_MONITOR_t)) {
                pid_response_receive_count++;
                pid_request_cmd_vel_count = 2;

                memcpy(&curr_pid_robot_monitor, pRcvData, sizeof(PID_ROBOT_MONITOR_t));

                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.lTempPosi_x);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.lTempPosi_y);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.sTempTheta);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.battery_percent);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.byUS1);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.byUS2);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.byUS3);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.byUS4);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "0x%x", static_cast<uint8_t>(curr_pid_robot_monitor.byPlatStatus.val));
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.linear_velocity);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "%d", curr_pid_robot_monitor.angular_velocity);
                RCLCPP_INFO(rclcpp::get_logger("SerialComm"), " ");

                if (robotParamData.use_MDUI == 1) {
                    MakeMDRobotMessage2(&curr_pid_robot_monitor);
                    PubRobotOdomMessage();
                }
            }
            break;
        }

        case PID_ROBOT_PARAM:
        {
            if (byRcvDataSize == sizeof(PID_ROBOT_PARAM_t)) {
                PID_ROBOT_PARAM_t* p = reinterpret_cast<PID_ROBOT_PARAM_t*>(pRcvData);
                // 디버그용 출력 원할 시 #if 0 → 1 로 변경
            }
            break;
        }

        default:
            break;
    }

    return 1;
}

int MdMotorDriverNode::AnalyzeReceivedData(uint8_t byArray[], uint8_t byBufNum)
{
    uint8_t i, j;
    uint8_t data;

    if (byPacketNum >= MAX_PACKET_SIZE) {
        rcv_step = 0;
        byPacketNum = 0;
        return 0;
    }

    for (j = 0; j < byBufNum; j++) {
        data = byArray[j];

#if (ENABLE_SERIAL_DEBUG == 1)
        printf("%02x(%3d) ", data, data);
#endif

        switch (rcv_step)
        {
            case 0: // Check IDPC
                if (data == robotParamData.nIDPC) {
                    byPacketNum = 0;
                    byChkSum = data;
                    serial_comm_rcv_buff[byPacketNum++] = data;
                    rcv_step++;
                } else {
                    byPacketNum = 0;
#if (ENABLE_SERIAL_DEBUG == 1)
                    RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "received ID: %d(0x%02x)", data, data);
                    RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "error.ser: %s, %d", __FILE__, __LINE__);
#endif
                }
                break;

            case 1: // Check source ID (MDUI/MDT)
                if (data == robotParamData.nIDMDUI || data == robotParamData.nIDMDT) {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;
                    rcv_step++;
                } else {
                    rcv_step = 0;
                    byPacketNum = 0;
#if (ENABLE_SERIAL_DEBUG == 1)
                    RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "error.ser: %s, %d", __FILE__, __LINE__);
#endif
                }
                break;

            case 2: // Device ID (1 or broadcast)
                if (data == 1 || data == ID_ALL) {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;
                    rcv_step++;
                } else {
                    rcv_step = 0;
                    byPacketNum = 0;
#if (ENABLE_SERIAL_DEBUG == 1)
                    RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "error.ser: %s, %d", __FILE__, __LINE__);
#endif
                }
                break;

            case 3: // PID
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;
                rcv_step++;
                break;

            case 4: // Data Length
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;
                byMaxDataNum = data;
                byDataNum = 0;
                rcv_step++;
                break;

            case 5: // Payload
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                if (++byDataNum >= MAX_DATA_SIZE) {
                    rcv_step = 0;
#if (ENABLE_SERIAL_DEBUG == 1)
                    RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "error.ser: %s, %d", __FILE__, __LINE__);
#endif
                    break;
                }

                if (byDataNum >= byMaxDataNum) {
                    rcv_step++;
                }
                break;

            case 6: // Checksum
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                if (byChkSum == 0) {
#if (ENABLE_SERIAL_DEBUG == 1)
                    printf("\r\n");
#endif
                    MdReceiveProc();
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "Error.Checksum");
                }

                rcv_step = 0;
                byPacketNum = 0;
                break;

            default:
                rcv_step = 0;
                byPacketNum = 0;
                break;
        }
    }

    return 1;
}

void MdMotorDriverNode::ReceiveSerialData()
{
    uint8_t byRcvBuf[250];
    uint8_t byBufNumber = 0;

    byBufNumber = ser->available();  // struct 멤버 ser 사용 //ser method not coded 

    if (byBufNumber != 0)
    {
        if (byBufNumber > sizeof(byRcvBuf)) {
            byBufNumber = sizeof(byRcvBuf);
        }

        size_t bytesRead = ser->read(byRcvBuf, byBufNumber); //ser method not coded

        memcpy(tempBuffer, byRcvBuf, bytesRead);
        tempLength = bytesRead;

        AnalyzeReceivedData(tempBuffer, tempLength);
    }
}

void MdMotorDriverNode::run()
{
    using namespace std::chrono_literals;

    if (robotParamData.use_MDUI == 1) {
        robotParamData.nRMID = robotParamData.nIDMDUI;
        RCLCPP_INFO(this->get_logger(), " Using MDUI(RS232)");
    } else {
        robotParamData.nRMID = robotParamData.nIDMDT;
        RCLCPP_INFO(this->get_logger(), " Direct MDT(RS485)");
    }

    robotParamData.nDiameter = static_cast<int>(robotParamData.wheel_radius * 2.0 * 1000.0);

    // Serial 초기화
    if (!ser->open(robotParamData.serial_port, robotParamData.nBaudrate)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        rclcpp::shutdown();
        return;
    }

    rclcpp::Rate init_loop_rate(10);
    auto start_time = now();
    auto start_delay = 0.5;

    // Start delay: 0.5s
    while (rclcpp::ok()) {
        if ((now() - start_time).seconds() >= start_delay) break;
        ReceiveSerialData();
        rclcpp::spin_some(std::enable_shared_from_this<MdMotorDriverNode>::shared_from_this());
        init_loop_rate.sleep();
    }

    byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
    check_connection_retry_count = 0;
    fgInitsetting = INIT_SETTING_STATE_NONE;

    while (rclcpp::ok()) {
        ReceiveSerialData();
        InitMotorParameter();
        if (fgInitsetting != INIT_SETTING_STATE_NONE) break;
        rclcpp::spin_some(std::enable_shared_from_this<MdMotorDriverNode>::shared_from_this());
        init_loop_rate.sleep();
    }

    if (fgInitsetting == INIT_SETTING_STATE_OK) {
        RCLCPP_INFO(this->get_logger(), "[Init done]\r\n");
        mdui_mdt_connection_state = true;
        remote_pc_connection_state = false;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error. Init ROBOT");
        rclcpp::shutdown();
        return;
    }

    pid_request_cmd_vel_count = 0;
    byCntComStep = 0;

    rclcpp::Rate request_loop_rate(10);

    bool old_mdui_mdt_connection_state = false;
    bool old_remote_pc_connection_state = false;

    while (rclcpp::ok()) {
        if (!remote_pc_connection_state && old_remote_pc_connection_state) {
            RCLCPP_INFO(this->get_logger(), "Remote PC connection state: error!!!\r\n");
            old_remote_pc_connection_state = remote_pc_connection_state;
            pid_request_cmd_vel_count = 0;
        } else if (remote_pc_connection_state && !old_remote_pc_connection_state) {
            RCLCPP_INFO(this->get_logger(), "Remote PC connection state: Ok\r\n");
            old_remote_pc_connection_state = remote_pc_connection_state;
            pid_request_cmd_vel_count = 0;
        }

        if (!mdui_mdt_connection_state && old_mdui_mdt_connection_state) {
            RCLCPP_INFO(this->get_logger(), robotParamData.use_MDUI ? "MDUI connection state: error!\r\n" : "MDT connection state: error!\r\n");
            old_mdui_mdt_connection_state = mdui_mdt_connection_state;
        } else if (mdui_mdt_connection_state && !old_mdui_mdt_connection_state) {
            RCLCPP_INFO(this->get_logger(), robotParamData.use_MDUI ? "MDUI connection state: Ok\r\n" : "MDT connection state: Ok\r\n");
            old_mdui_mdt_connection_state = mdui_mdt_connection_state;
        }

        ReceiveSerialData();
        RequestRobotStatus();
        rclcpp::spin_some(std::enable_shared_from_this<MdMotorDriverNode>::shared_from_this());
        request_loop_rate.sleep();
    }
}

void MdMotorDriverNode::velCmdRcvTimeoutCallback()
{
    static uint32_t old_velCmdRcvCount = 0;
    static uint32_t old_pid_response_receive_count = 0;

    if (velCmdRcvCount == old_velCmdRcvCount)
    {
        goal_cmd_speed = 0.0;
        goal_cmd_ang_speed = 0.0;

        if (remote_pc_connection_state == true)
        {
            velCmdUpdateCount++;
            remote_pc_connection_state = false;

#ifdef ENABLE_MD_MESSAGE
            RCLCPP_WARN(this->get_logger(), "Remote PC connection lost (cmd_vel timeout)");
#endif
        }
    }
    else
    {
        old_velCmdRcvCount = velCmdRcvCount;

        if (remote_pc_connection_state == false)
        {
            remote_pc_connection_state = true;

#ifdef ENABLE_MD_MESSAGE
            RCLCPP_INFO(this->get_logger(), "Remote PC connection restored (cmd_vel OK)");
#endif
        }
    }

    if (pid_request_cmd_vel_count > 5)
    {
        if (mdui_mdt_connection_state == true)
        {
            mdui_mdt_connection_state = false;

#ifdef ENABLE_MD_MESSAGE
            RCLCPP_WARN(this->get_logger(), "MDUI/MDT connection lost (serial interface error)");
#endif
        }
    }
    else if (pid_request_cmd_vel_count == 2)
    {
        if (mdui_mdt_connection_state == false)
        {
            mdui_mdt_connection_state = true;

#ifdef ENABLE_MD_MESSAGE
            RCLCPP_INFO(this->get_logger(), "MDUI/MDT connection restored");
#endif
        }
    }
}

void MdMotorDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr keyVel)
{
    if (fgInitsetting == INIT_SETTING_STATE_OK)
    {
        velCmdRcvCount++;
        velCmdUpdateCount++;

        goal_cmd_speed = keyVel->linear.x;
        goal_cmd_ang_speed = keyVel->angular.z;

#ifdef ENABLE_MD_MESSAGE
        RCLCPP_INFO(this->get_logger(), "Goal cmd_vel (m/sec): lin: %.2f, ang: %.2f", keyVel->linear.x, keyVel->angular.z);
#endif
    }
}

void MdMotorDriverNode::resetPositionCallback(const std_msgs::msg::Bool::SharedPtr reset_position_msg)
{
    if (reset_position_msg->data == 1) {
        RCLCPP_INFO(this->get_logger(), "Reset Position");
        reset_pos_flag = true;
    }
}

void MdMotorDriverNode::resetAlarmCallback(const std_msgs::msg::Bool::SharedPtr reset_alarm_msg)
{
    if (reset_alarm_msg->data == 1) {
        RCLCPP_INFO(this->get_logger(), "Reset Alarm");
        reset_alarm_flag = true;
    }
}
