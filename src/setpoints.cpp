#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <stdint.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace nav_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control"), armed_(false)
    {
        // Publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Define QoS profile for the subscriber
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        
        // Create subscriber for vehicle control mode (to check the armed status)
        arming_status_subscriber_ = this->create_subscription<VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", qos_profile, [this](const VehicleControlMode::SharedPtr msg) {
                armed_ = msg->flag_armed;
                if (armed_) {
                    RCLCPP_INFO(this->get_logger(), "Drone is armed.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Drone is disarmed.");
                }
            });

        // Subscribe to the Path topic
        path_subscriber_ = this->create_subscription<Path>(
            "/in/trajectory_path", qos_profile, [this](const Path::SharedPtr msg) {
                if (!msg->poses.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Received Path with %zu poses", msg->poses.size());

                    // Extract the first pose from the Path
                    const auto &first_pose = msg->poses[0];

                    // Convert PoseStamped to TrajectorySetpoint
                    TrajectorySetpoint setpoint_msg{};
                    setpoint_msg.position = {
                        static_cast<float>(first_pose.pose.position.y), // y
                        static_cast<float>(first_pose.pose.position.x), // x
                        static_cast<float>(-first_pose.pose.position.z) // -z
                    };

                    // Extract yaw from the quaternion and add M_PI / 2.0
                    tf2::Quaternion q(
                        first_pose.pose.orientation.x,
                        first_pose.pose.orientation.y,
                        first_pose.pose.orientation.z,
                        first_pose.pose.orientation.w);
                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);
                    setpoint_msg.yaw = static_cast<float>(-yaw + M_PI / 2.0); // Adjust yaw to pegasus

                    // Publish the TrajectorySetpoint
                    publish_offboard_control_mode();
                    trajectory_setpoint_publisher_->publish(setpoint_msg);
                    RCLCPP_INFO(this->get_logger(), "Published Trajectory Setpoint to first Path pose");

                    // Arm the drone if it's not armed
                    if (!armed_) {
                        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                        arm();
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Received empty Path message");
                }
            });
    }

    void arm();
    void disarm();

private:
    bool armed_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleControlMode>::SharedPtr arming_status_subscriber_;
    rclcpp::Subscription<Path>::SharedPtr path_subscriber_;

    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published Offboard Control Mode");
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
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

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OffboardControl>();
    RCLCPP_INFO(node->get_logger(), "Starting Offboard Control Node...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}