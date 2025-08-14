#include "px4_msg_converter_node.hpp"
#include "px4_offboard_control.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Create your nodes
    auto converter_node = std::make_shared<osep_simulation_environment::PX4MsgConverterNode>();
    auto offboard_node = std::make_shared<osep_simulation_environment::PX4OffboardControl>();
    // Spin both nodes (multi-threaded executor if needed)
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(converter_node);
    exec.add_node(offboard_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
