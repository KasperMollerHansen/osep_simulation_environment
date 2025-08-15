#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include "px4_msg_converter.hpp"

namespace osep_simulation_environment {

class PX4MsgConverterNode : public rclcpp::Node {
public:
    PX4MsgConverterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("trajectory_setpoint_converter_node", options)
    {
        sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/osep/test", 10,
            std::bind(&PX4MsgConverterNode::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        auto converted = PX4MsgConverter::convert(*msg);
        RCLCPP_INFO(this->get_logger(), "Converted TrajectorySetpoint: [%.2f, %.2f, %.2f]",
            converted.position[0], converted.position[1], converted.position[2]);
        // Add further processing or publishing here if needed
    }

    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_;
};

} // namespace osep_simulation_environment