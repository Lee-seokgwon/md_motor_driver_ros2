cmake_minimum_required(VERSION 3.8)
project(md_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(md_motor_interface_msgs REQUIRED)  # MD custom ros interface package
#Serial package must be added after testing the library
#Serial package must be added after testing the library

# include 디렉토리 추가
include_directories(
  include
)

#실행 가능한 바이너리 파일을 생성
#바이너리는 main.cpp, com.cpp, robot.cpp를 컴파일해서 만들어라
#.hpp(헤더)는 포함할 필요 없음 — 컴파일러가 #include를 통해 자동 탐지
add_executable(md_motor_driver_node
  src/main.cpp
  src/com.cpp
  src/robot.cpp
)

# 의존성 연결
ament_target_dependencies(md_motor_driver_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  md_motor_interface_msgs
)

#설치를 해야 ROS2 시스템이 이 바이너리를 인식하고 ros2 run, ros2 launch 등에 사용할 수 있음
#colcon build --symlink-install로 빌드할 경우, 설치된 것처럼 링크만 걸지만 여전히 install() 명령은 필요함
install(TARGETS md_motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
