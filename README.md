# 🦾 MD 모터 드라이버 ROS 2 패키지 (Prototyping)

**본 패키지의 원본 (ROS1 버전의 MD ROS드라이버)의 저작권은 MDROBOT 사에 있습니다.**
**https://www.mdrobot.co.kr/**

본 패키지는 MD 시리즈 모터 드라이버를 ROS 2 (Humble) 환경에서 제어할 수 있도록 포팅한 **프로토타입 버전**입니다.  
⚠️ **아직 실제 하드웨어 연동 및 검증은 진행되지 않았습니다.**

---


## 개발환경

- OS: Ubuntu 22.04 (⚠️ Boost::asio 사용을 위해 **WSL/VM보다는 네이티브 우분투** 권장)
- ROS 2 Humble
- C++17
- Boost.Asio (libboost-system-dev)

## 🚀 실행 방법

```bash
# 홈 디렉토리에서 작업 시작
cd ~
mkdir md
cd md

# 패키지 클론
git clone https://github.com/Lee-seokgwon/md_motor_driver_ros2.git

# 빌드
cd md_motor_driver_ros2
colcon build --symlink-install

# 환경 설정
source install/setup.bash

# 노드 실행
ros2 launch md_motor_driver md_robot_node_launch.py
```

## ⚙️ 파라미터 수정 방법

```bash
cd ~/md/md_motor_driver_ros2/src/md_motor_driver/launch
gedit md_robot_node_launch.py
```


수정 가능한 파라미터 예시:
```python
'serial_port': '/dev/ttyUSB0',
'serial_baudrate': 57600,
'wheel_radius': 0.0935,
'wheel_length': 0.454,
'maxrpm': 1000,
'use_MDUI': 0,
...
```

파라미터 수정후 재빌드
```bash
cd ~/md/md_motor_driver_ros2
colcon build --symlink-install
source install/setup.bash
```

📌 주의 사항

실행 시 실제 하드웨어(MDT, RS485 등)가 연결되어 있지 않으면 시리얼 포트 에러가 발생합니다.

본 코드는 아직 HW 실검증이 되지 않은 상태입니다.
