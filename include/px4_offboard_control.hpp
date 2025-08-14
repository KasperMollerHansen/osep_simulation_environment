#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <nav_msgs/msg/path.hpp>

namespace osep_simulation_environment {

class PX4OffboardControl : public rclcpp::Node {
public:
    PX4OffboardControl();
    void arm();
    void disarm();

private:
    bool armed_ = false;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr arming_status_subscriber_;
    rclcpp::Subscription<osep_simulation_environment::msg::Action>::SharedPtr action_subscriber_;

    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

} // namespace osep_simulation_environment
