#include <rclcpp/rclcpp.hpp>
#include "md_motor_driver/md_motor_driver_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MdMotorDriverNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}