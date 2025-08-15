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
        this->declare_parameter<std::string>("input_vel_cmd", "/osep/vel_cmd");
        std::string topic_name = this->get_parameter("input_vel_cmd").as_string();

        sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            topic_name, 10,
            std::bind(&PX4MsgConverterNode::callback, this, std::placeholders::_1)
        );

        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10
        );
    }

private:
    void callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        auto converted = PX4MsgConverter::convert(*msg);
        trajectory_setpoint_publisher_->publish(converted);
    }

    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
};

} // namespace osep_simulation_environment