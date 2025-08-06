#include "md_controller/com.hpp"
#include "md_controller/kinematics.hpp"

#include "geometry_msgs/msg/twist.hpp"

Communication Com;  
MotorVar Motor;

geometry_msgs::msg::TransformStamped odom_tf;
sensor_msgs::msg::JointState joint_states;

BYTE SendCmdRpm = OFF;
int left_rpm_ = 0;
int right_rpm_ = 0;

void CmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // cmd_vel에서 linear.x와 angular.z 추출
    float linear_x = msg->linear.x;   // 직진 속도 (m/s)
    float angular_z = msg->angular.z; // 회전 속도 (rad/s)
    cmdVelToRpm(linear_x, angular_z, left_rpm_, right_rpm_); //convert /cmd_vel to left_rpm_ , right_rpm_
    SendCmdRpm = ON;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("md_controller_node");

    // create TF
    rclcpp::Time stamp_now;
    tf2_ros::TransformBroadcaster tf_broadcaster_(node);

    //subscriber
    auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, CmdVelCallBack);

    //publisher (rpm, encoder tick...maybe?)
    auto motor1_enc_pub = node->create_publisher<std_msgs::msg::Int32>("/mdh250_l_en", 10);
    auto motor2_enc_pub = node->create_publisher<std_msgs::msg::Int32>("/mdh250_r_en", 10);
    auto motor1_rpm_pub = node->create_publisher<std_msgs::msg::Int32>("/mdh250_l_rpm", 10);
    auto motor2_rpm_pub = node->create_publisher<std_msgs::msg::Int32>("/mdh250_r_rpm", 10);

    //Motor driver settup-------------------------------------------------------------------------------
    node->declare_parameter("MDUI", 184);
    node->declare_parameter("MDT", 183);
    node->declare_parameter("Port", "/dev/ttyMotor");
    node->declare_parameter("Baudrate", 57600);
    node->declare_parameter("ID", 1); //fix
    node->declare_parameter("GearRatio", 15);
    node->declare_parameter("poles", 10);
    node->declare_parameter("wheel_radius", 0.033);
    node->declare_parameter("wheel_base", 0.16);

    node->get_parameter("MDUI", Com.nIDMDUI);
    node->get_parameter("MDT", Com.nIDMDT);
    node->get_parameter("Port", Com.nPort);
    node->get_parameter("Baudrate", Com.nBaudrate);
    node->get_parameter("ID", Motor.ID);
    node->get_parameter("GearRatio", Motor.GearRatio);
    node->get_parameter("poles", Motor.poles);

    //Robot Param load for kinematics.cpp
    float wheel_radius_param, wheel_base_param;
    node->get_parameter("wheel_radius", wheel_radius_param);
    node->get_parameter("wheel_base", wheel_base_param);
    setRobotParams(wheel_radius_param, wheel_base_param);

    Motor.PPR       = Motor.poles*3*Motor.GearRatio;           //poles * 3(HALL U,V,W) * gear ratio
    Motor.Tick2RAD  = (360.0/Motor.PPR)*PI / 180;

    IByte iData, left_iData, right_iData; //left_iData, right_iData for dual motor

    int nArray[20];
    static BYTE fgInitsetting, byCntInitStep, byCntComStep, byCnt2500us, byCntStartDelay, byCntCase[5];
    
    byCntInitStep     = 1;
    Motor.InitMotor   = ON;
    fgInitsetting     = OFF;
    Motor.InitError   = 0;
    Motor.last_rad    = 0;
    Motor.last_tick   = 0;

    InitSerial();   //communication initialization in com.cpp
    while (rclcpp::ok()) {
        
        ReceiveDataFromController(Motor.InitMotor);
        if(++byCnt2500us == 50)
        {
            byCnt2500us = 0;
            
            if(fgInitsetting == ON)
            {
                switch(++byCntComStep)
                {
                case 1:{ //create tf & update motor position //maybe not needed. change this logic to get odom
                    geometry_msgs::msg::TransformStamped transformStamped;
                    transformStamped.header.stamp = node->now();
                    transformStamped.header.frame_id = "world";
                    transformStamped.child_frame_id = "motor_joint";

                    transformStamped.transform.translation.x = 0.0;
                    transformStamped.transform.translation.y = 0.0;
                    transformStamped.transform.translation.z = 0.15;

                    Motor.current_tick     = Com.position;

                    Motor.last_diff_tick   = Motor.current_tick - Motor.last_tick;
                    Motor.last_tick        = Motor.current_tick;
                    Motor.last_rad        += Motor.Tick2RAD * (double)Motor.last_diff_tick;
                    // printf("%f\n", Motor.Tick2RAD);

                    tf2::Quaternion q;
                    q.setRPY(0, 0, -Motor.last_rad);
                    transformStamped.transform.rotation.x = q.x();
                    transformStamped.transform.rotation.y = q.y();
                    transformStamped.transform.rotation.z = q.z();
                    transformStamped.transform.rotation.w = q.w();

                    tf_broadcaster_.sendTransform(transformStamped);

                    // 모터 엔코더 값 퍼블리시
                    std_msgs::msg::Int32 motor1_enc_msg;
                    std_msgs::msg::Int32 motor2_enc_msg;
                    std_msgs::msg::Int32 motor1_rpm_msg;
                    std_msgs::msg::Int32 motor2_rpm_msg;
                    
                    motor1_enc_msg.data = Com.motor1_position;
                    motor2_enc_msg.data = Com.motor2_position;
                    
                    motor1_enc_pub->publish(motor1_enc_msg);
                    motor2_enc_pub->publish(motor2_enc_msg);

                    // 모터 rpm 값 퍼블리시
                    std_msgs::msg::Int32 motor1_rpm_msg;
                    std_msgs::msg::Int32 motor2_rpm_msg;

                    motor2_rpm_msg.data = Com.motor1_rpm;
                    motor2_rpm_msg.data = Com.motor2_rpm;

                    motor1_rpm_pub->publish(motor1_rpm_msg);
                    motor2_rpm_pub->publish(motor2_rpm_msg);

                    auto end = std::chrono::high_resolution_clock::now();
                    break;
                }
                case 2: //Control motor & request motor info
                    if(++byCntCase[byCntComStep] == TIME_100MS)
                    {
                        byCntCase[byCntComStep] = 0;

                        if(SendCmdRpm)
                        {
                            left_iData = Short2Byte(left_rpm_ * Motor.GearRatio); // #1,2 Wheel RPM
                            right_iData = Short2Byte(right_rpm_ * Motor.GearRatio);

                            nArray[0] = 1; //left motor enable
                            nArray[1] = left_iData.byLow; //left motor rpm (lower 8bits)
                            nArray[2] = left_iData.byHigh; //left motor rpm (higher 8bits)
                            nArray[3] = 1; //right motor enable
                            nArray[4] = right_iData.byLow; //right motor rpm (lower 8bits)
                            nArray[5] = right_iData.byHigh; //right motor rpm (higher 8bits)
                            nArray[6] = 1; // PID_PNT_MONITOR return

                            PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray); //dual channel motor controller -> pid 207 (md ros manual)

                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);  // Main data request

                            SendCmdRpm = OFF;
                        }
                        else
                        {
                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);  // Main data request
                            //------------------------------------------------------------------
                        }
                        
                    }
                    byCntComStep=0;
                    break;  
                }
            }
            else
            {
                if(byCntStartDelay <= 200) byCntStartDelay++;
                else
                {
                    switch (byCntInitStep)
                    {
                    case 1: //Motor connect check
                        nArray[0] = PID_MAIN_DATA;
                        PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);

                        if(Motor.InitMotor == ON)
                            Motor.InitError++;
                        else
                            byCntInitStep++;

                        if(Motor.InitError > 10){
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"ID %d MOTOR INIT ERROR!!", Motor.ID); 
                            return 0;
                        }
                    
                        break;
                    case 2:
                        byCntInitStep++;
                        break;

                    case 3: //Motor torque ON
                        nArray[0] = 0;
                        nArray[1] = 0;
                        PutMdData(PID_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);

                        byCntInitStep++;
                        break;

                    case 4: //Motor POS reset
                        nArray[0] = 0;
                        PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor.ID, nArray);
                        byCntInitStep++;
                        break;

                    case 5:
                        printf("========================================================\n\n");
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MOTOR INIT END\n");
                        fgInitsetting = ON;

                        break;

                    }
                }
            }
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
