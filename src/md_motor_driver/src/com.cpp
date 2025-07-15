#include "md_motor_driver/com.hpp"
#include "md_motor_driver/main.hpp"
#include "md_motor_driver/global.hpp"
#include "md_motor_driver/SerialComm.hpp"

//test logging code
void com_test_log() {
  RCLCPP_INFO(rclcpp::get_logger("com"), "✅ com.cpp compiled and linked");
}

//boost::asio 이용하여 시리얼 통신 구현 예정 (기존 ros-noetic-serial 은 humble 버전 없음)
//boost 라이브러리는 22.04에 기본 포함되어있음.

#define MD_PROTOCOL_POS_PID             3
#define MD_PROTOCOL_POS_DATA_LEN        4
#define MD_PROTOCOL_POS_DATA_START      5

#define ENABLE_SERIAL_DEBUG             0

SerialComm serial_comm;

// 수신/송신 버퍼
uint8_t serial_comm_rcv_buff[MAX_PACKET_SIZE];
uint8_t serial_comm_snd_buff[MAX_PACKET_SIZE];

PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
PID_IO_MONITOR_t curr_pid_io_monitor;
PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;

extern uint32_t pid_response_receive_count;
extern uint32_t pid_request_cmd_vel_count;
extern INIT_SETTING_STATE_t fgInitsetting;

extern void MakeMDRobotMessage1(PID_PNT_MAIN_DATA_t *pData);
extern void MakeMDRobotMessage2(PID_ROBOT_MONITOR_t *pData);
extern void PubRobotOdomMessage(void);
extern void PubRobotRPMMessage(void);

extern std::string serial_port;

int InitSerialComm(void)
{
    std::string port = "/dev/" + serial_port;
    RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "Serial port: %s", port.c_str());
    RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "Serial baudrate: %d", robotParamData.nBaudrate);

    bool ok = serial_comm.open(port, robotParamData.nBaudrate);

    if (ok && serial_comm.isOpen()) {
        RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "*Serial port open success!");
        return 1;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "Unable to open serial port");
        return -1;
    }
}

uint8_t CalCheckSum(uint8_t *pData, uint16_t length)
{
    uint16_t sum;
    sum = 0;

    for(int i = 0; i < length; i++) {
        sum += *pData++;
    }

    sum = ~sum + 1; //check sum
    return sum;
}

int PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length)
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
    if (serial_comm.isOpen()) {
        serial_comm.writeData(serial_comm_snd_buff, len);
    }

    return 1;
}

int MdReceiveProc(void)
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

int AnalyzeReceivedData(uint8_t byArray[], uint8_t byBufNum)
{
    uint8_t i, j;
    uint8_t data;
    static uint8_t byChkSec;
    static long lExStampSec, lExStampNsec;
    static uint32_t byPacketNum = 0;
    static uint32_t rcv_step = 0;
    static uint8_t byChkSum = 0;
    static uint16_t byMaxDataNum = 0;
    static uint16_t byDataNum = 0;

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

int ReceiveSerialData(SerialComm& serial)
{
    uint8_t byRcvBuf[250];
    uint8_t byBufNumber = 0;

    static uint8_t tempBuffer[250];
    static uint8_t tempLength = 0;

    byBufNumber = serial.available();  // available() 메서드 구현 필요

    if (byBufNumber != 0)
    {
        if (byBufNumber > sizeof(byRcvBuf)) {
            byBufNumber = sizeof(byRcvBuf);
        }

        size_t bytesRead = serial.readData(byRcvBuf, byBufNumber);

        memcpy(tempBuffer, byRcvBuf, bytesRead);
        tempLength = bytesRead;

        AnalyzeReceivedData(tempBuffer, tempLength);
    }

    return 1;
}
