
#include "px4_offboard_control.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace osep_simulation_environment {

PX4OffboardControl::PX4OffboardControl() : rclcpp::Node("offboard_control"), armed_(false)
{
    // Publishers
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    // Define QoS profile for the subscriber
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Create subscriber for vehicle control mode (to check the armed status)
    arming_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode", qos_profile, [this](const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
            armed_ = msg->flag_armed;
            if (armed_) {
                RCLCPP_INFO(this->get_logger(), "Drone is armed.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Drone is disarmed.");
            }
        });

    // Subscribe to Action topic. Arm if any Action message is received.
    action_subscriber_ = this->create_subscription<osep_simulation_environment::msg::Action>(
        "/vel_controller/action", qos_profile, [this](const osep_simulation_environment::msg::Action::SharedPtr /*msg*/) {
            if (!armed_) {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                arm();
            }
        });
}

void PX4OffboardControl::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void PX4OffboardControl::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void PX4OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published Vehicle Command: %d", command);
}

} // namespace osep_simulation_environment
