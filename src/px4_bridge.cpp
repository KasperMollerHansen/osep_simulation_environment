#include <rclcpp/rclcpp.hpp>
#include "px4_msg_converter_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<osep_simulation_environment::PX4MsgConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}