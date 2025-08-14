
#include <rclcpp/rclcpp.hpp>
#include "osep_simulation_environment/msg/action.hpp"
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include "px4_msg_converter.hpp"

class PX4MsgConverterNode : public rclcpp::Node {
public:
	PX4MsgConverterNode() : Node("px4_msg_converter_node") {
		this->declare_parameter<std::string>("action_topic", "/vel_controller/action");
		std::string action_topic = this->get_parameter("action_topic").as_string();
		setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		action_sub_ = this->create_subscription<osep_simulation_environment::msg::Action>(
			action_topic, 10,
			[this](osep_simulation_environment::msg::Action::UniquePtr msg) {
				auto setpoint = osep_simulation_environment::PX4MsgConverter::convert(*msg);
				setpoint_pub_->publish(setpoint);
			}
		);
		RCLCPP_INFO(this->get_logger(), "Listening to %s, publishing PX4 setpoints", action_topic.c_str());
	}
private:
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
	rclcpp::Subscription<osep_simulation_environment::msg::Action>::SharedPtr action_sub_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PX4MsgConverterNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}