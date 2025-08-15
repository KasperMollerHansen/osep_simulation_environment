#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

namespace osep_simulation_environment {

class PX4MsgConverterNode : public rclcpp::Node {
public:
    explicit PX4MsgConverterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    bool is_armed_ = false;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr arming_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    void callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);
    void publish_offboard_control_mode(const px4_msgs::msg::TrajectorySetpoint &msg);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void arm();
};

} // namespace osep_simulation_environment