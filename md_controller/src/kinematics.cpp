#include "md_controller/kinematics.hpp"
#include <cmath>

#ifndef PI
#define PI 3.14159265359
#endif

float wheel_radius = 0.0535;  // default 바퀴 반지름 (m), mdh250 = 0.103, mdh80=0.0535  
float wheel_base = 0.35;     // default 좌우 바퀴 간격 (m)

void setRobotParams(float radius, float base)
{
    wheel_radius = radius;
    wheel_base = base;
}


void cmdVelToRpm(float linear_x, float angular_z, int& left_rpm, int& right_rpm)
{
    // 직선 속도와 회전 속도를 20%로 줄이기
    float linear_x_scaled = linear_x * 0.2;  // 20%로 줄이기
    float angular_z_scaled = angular_z * 0.2;  // 20%로 줄이기
    
    float left_vel = linear_x_scaled - (angular_z_scaled * wheel_base / 2.0);
    float right_vel = linear_x_scaled + (angular_z_scaled * wheel_base / 2.0);
    
    // 선속도를 RPM으로 변환
    left_rpm = static_cast<int>(std::round((left_vel * 60.0) / (2.0 * PI * wheel_radius)));
    right_rpm = static_cast<int>(std::round((right_vel * 60.0) / (2.0 * PI * wheel_radius)));
    
    // RPM 제한
    const int max_rpm = 300;  // 최대 RPM 설정
    left_rpm = std::min(left_rpm, max_rpm);
    right_rpm = std::min(right_rpm, max_rpm);
    
    // 디버깅용 출력 (필요시 주석 해제)
    // printf("linear_x: %.3f, angular_z: %.3f -> left_rpm: %d, right_rpm: %d\n", 
    //        linear_x, angular_z, left_rpm, right_rpm);
}
