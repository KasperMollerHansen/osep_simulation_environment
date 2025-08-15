#include "px4_msg_converter_node.hpp"
#include "px4_msg_converter.hpp"
#include <cmath>

namespace osep_simulation_environment {

PX4MsgConverterNode::PX4MsgConverterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_setpoint_converter_node", options)
{
    this->declare_parameter<std::string>("input_vel_cmd", "/osep/vel_cmd");
    std::string topic_name = this->get_parameter("input_vel_cmd").as_string();

    // Define QoS profile for the subscriber
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Publishers
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // Subscriber for arming status
    arming_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode", qos_profile,
        [this](const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
            is_armed_ = msg->flag_armed;
        });

    // Subscriber for incoming trajectory setpoints
    sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        topic_name, qos_profile,
        std::bind(&PX4MsgConverterNode::callback, this, std::placeholders::_1)
    );
}

void PX4MsgConverterNode::callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
    publish_offboard_control_mode(*msg);
    auto converted = PX4MsgConverter::convert(*msg);
    trajectory_setpoint_publisher_->publish(converted);

    if (!is_armed_) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // Set to offboard mode
        arm();
    }
}

void PX4MsgConverterNode::publish_offboard_control_mode(const px4_msgs::msg::TrajectorySetpoint &msg) {
    px4_msgs::msg::OffboardControlMode mode_msg{};
    mode_msg.position = !(std::isnan(msg.position[0]) || std::isnan(msg.position[1]) || std::isnan(msg.position[2]));
    mode_msg.velocity = !(std::isnan(msg.velocity[0]) || std::isnan(msg.velocity[1]) || std::isnan(msg.velocity[2]));
    mode_msg.acceleration = !(std::isnan(msg.acceleration[0]) || std::isnan(msg.acceleration[1]) || std::isnan(msg.acceleration[2]));
    mode_msg.attitude = !std::isnan(msg.yaw);
    mode_msg.body_rate = !std::isnan(msg.yawspeed);
    mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(mode_msg);
}

void PX4MsgConverterNode::publish_vehicle_command(uint16_t command, float param1, float param2) {
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
}

void PX4MsgConverterNode::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

} // namespace osep_simulation_environment

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<osep_simulation_environment::PX4MsgConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}