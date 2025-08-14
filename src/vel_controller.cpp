#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen3/Eigen/Dense> // For Eigen::Vector3d
#include <cmath>
#include <chrono>
#include <iostream>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace nav_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control"), armed_(false)
    {
        this->declare_parameter<double>("interpolation_distance", 2.0);
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<double>("max_velocity", 10.0);
        this->declare_parameter<double>("min_velocity", 1.0);
        this->declare_parameter<double>("max_acceleration", 0.5);
        this->declare_parameter<double>("max_angle_change", M_PI / 6.0); // 30 degrees
        this->declare_parameter<double>("max_yaw_to_velocity_angle", M_PI / 1.8); // 100 degrees per second
        this->declare_parameter<std::string>("path_topic", "/planner/smoothed_path");

        interpolation_distance_ = this->get_parameter("interpolation_distance").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        min_velocity_ = this->get_parameter("min_velocity").as_double();
        max_acceleration_ = this->get_parameter("max_acceleration").as_double();
        max_angle_change_ = this->get_parameter("max_angle_change").as_double();
        max_yaw_to_velocity_angle_ = this->get_parameter("max_yaw_to_velocity_angle").as_double();
        std::string path_topic = this->get_parameter("path_topic").as_string();

        // Publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // TF2 setup
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Define QoS profile for the subscriber
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // Create subscriber for vehicle control mode (to check the armed status)
        arming_status_subscriber_ = this->create_subscription<VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", qos_profile, [this](const VehicleControlMode::SharedPtr msg) {
                armed_ = msg->flag_armed;
                if (armed_) {
                    // RCLCPP_INFO(this->get_logger(), "Drone is armed.");
                } else {
                    // RCLCPP_INFO(this->get_logger(), "Drone is disarmed.");
                }
            });

        // Subscribe to the Path topic
        path_subscriber_ = this->create_subscription<Path>(
            path_topic, qos_profile, [this](const Path::SharedPtr msg) {
                process_path(msg);
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
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    void publish_offboard_control_mode_velocity();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void process_path(const Path::SharedPtr msg);
    std::string frame_id_;
    double interpolation_distance_;
    double max_velocity_;
    double min_velocity_;
    double max_acceleration_;
    double max_angle_change_;
    double max_yaw_to_velocity_angle_;
    double normalizeAngle(double angle);
    Eigen::Vector3d last_tf_pos;
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
 * @brief Publish the offboard control mode for velocity control.
 */
void OffboardControl::publish_offboard_control_mode_velocity()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Published Offboard Control Mode (Velocity)");
}

int countStraightLinePoints(const std::vector<geometry_msgs::msg::PoseStamped> &poses) {
    if (poses.size() < 3) {
        // Less than 3 points cannot form a line
        return poses.size();
    }

    int count = 2; // Start with the 2nd point
    Eigen::Vector3d prev_direction;

    for (size_t i = 2; i < poses.size() - 2; ++i) {
        // Calculate direction vectors
        Eigen::Vector3d dir1(
            poses[i].pose.position.x - poses[i - 2].pose.position.x,
            poses[i].pose.position.y - poses[i - 2].pose.position.y,
            poses[i].pose.position.z - poses[i - 2].pose.position.z);

        Eigen::Vector3d dir2(
            poses[i + 2].pose.position.x - poses[i].pose.position.x,
            poses[i + 2].pose.position.y - poses[i].pose.position.y,
            poses[i + 2].pose.position.z - poses[i].pose.position.z);

        // Normalize the direction vectors
        dir1.normalize();
        dir2.normalize();

        // Check if the direction vectors are collinear
        if ((dir1.cross(dir2)).norm() < 1e-1) { // Cross product close to zero
            count++;
        } else {
            break; // Stop counting if the line is broken
        }
    }

    return count + 1; // Include the last point in the count
}

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
    // RCLCPP_INFO(this->get_logger(), "Published Vehicle Command: %d", command);
}

// Helper function to normalize angles to the range [-π, π]
double OffboardControl::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void OffboardControl::process_path(const Path::SharedPtr msg)
{ 
    static Eigen::Vector3d previous_published_velocity(0.0, 0.0, 0.0); // Track the previous velocity
    static double previous_velocity = 0.0; // Track the previous speed
    static double previous_yaw = NAN; // Track the previous yaw
    static double dt = 1; // Time difference between updates (can be adjusted dynamically)
    static double delta_vel = min_velocity_ / 10.0; // Velocity change threshold
    static double delta_yaw = 0.1; // Yaw change threshold
    static double yaw_next = 0.0; // Initialize yaw_next

    // TF2 setup
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        // Get the transform from the drone's frame (e.g., base_link) to the world frame (e.g., map)
        transform_stamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform base_link to odom: %s", ex.what());
        return;
    }
    // Transform_pos
    Eigen::Vector3d pos_tf(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z);

    Eigen::Vector3d velocity_desired_world; // Declare the variable before the if-else block


    if (msg->poses.size() >= 2) { // Need at least two poses
        RCLCPP_INFO(this->get_logger(), "Received Path with %zu poses", msg->poses.size());

        last_tf_pos = pos_tf; // Update the last TF position

        int straight_line_points = std::max(1, countStraightLinePoints(msg->poses));
        RCLCPP_INFO(this->get_logger(), "Number of points on a straight line: %d", straight_line_points);
        double velocity = delta_vel * straight_line_points * delta_vel * straight_line_points * interpolation_distance_ / 2; // Hardcoded velocity
        
        // Clamp the velocity change to a
        if (velocity > previous_velocity + delta_vel*previous_velocity) {
            velocity = previous_velocity + delta_vel*previous_velocity;
        } else if (velocity < previous_velocity - delta_vel*previous_velocity) {
            velocity = previous_velocity - delta_vel*previous_velocity;
        }
        if (velocity > max_velocity_) {
            velocity = max_velocity_;
        } else if (velocity < min_velocity_) {
            velocity = min_velocity_;
        }
        previous_velocity = velocity;

        const geometry_msgs::msg::PoseStamped* pose_yaw = nullptr; // Declare a pointer to hold the selected pose
        // Get the first two poses
        const auto &pose0 = msg->poses[0];
        const auto &pose1 = msg->poses[1];

        // Calculate the index for posex
        int posex_index = std::clamp(straight_line_points / 2, 0, static_cast<int>(msg->poses.size() - 1));

        // Calculate velocity vector from pose0 to pose1
        // Transform pose1 to ned frame
        Eigen::Vector3d pos0(pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z);
        Eigen::Vector3d pos1(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z);
        
        if ((pos_tf - pos0).norm() > interpolation_distance_) {
            velocity_desired_world = (pos0 - pos_tf) / dt; // Move to pose0 if pose0 diverges from tf
            pose_yaw = &msg->poses[0];
        } else {
            velocity_desired_world = (pos1 - pos_tf) / dt; // Move to next pose
            pose_yaw = &msg->poses[1];
        }

        if (straight_line_points > 20){  //Quick fix to advoid yaw rotation
            pose_yaw = &msg->poses[posex_index]; // Use the pose at the calculated index
        }

        velocity_desired_world = velocity_desired_world / velocity_desired_world.norm() * velocity;

        // Check angular difference between previous and desired velocity
        if (previous_published_velocity.norm() > 1e-6) { // Ensure the previous velocity is not zero
            double dot_product = previous_published_velocity.normalized().dot(velocity_desired_world.normalized());
            double angle_change = std::acos(std::clamp(dot_product, -1.0, 1.0)); // Clamp to avoid numerical issues
        
            if (angle_change > max_angle_change_) {
                RCLCPP_WARN(this->get_logger(), "Angular change exceeds threshold. Adjusting velocity direction to max allowable angle.");
        
                // Calculate the axis of rotation (cross product of the two vectors)
                Eigen::Vector3d rotation_axis = previous_published_velocity.normalized().cross(velocity_desired_world.normalized());
                if (rotation_axis.norm() > 1e-6) { // Ensure the axis is valid
                    rotation_axis.normalize();
        
                    // Create a rotation matrix for the max allowable angle
                    Eigen::AngleAxisd rotation(max_angle_change_, rotation_axis);
        
                    // Rotate the previous velocity vector to the max allowable angle
                    velocity_desired_world = rotation * previous_published_velocity.normalized() * velocity;
                } else {
                    // If the rotation axis is invalid (vectors are parallel), keep the previous direction
                    velocity_desired_world = previous_published_velocity.normalized() * velocity;
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Previous velocity is zero. Skipping angular adjustment.");
        }
        // Calculate acceleration
        Eigen::Vector3d acceleration_desired_world = (velocity_desired_world - previous_published_velocity) / dt;

        // Limit the magnitude of the acceleration
        double acceleration_magnitude = acceleration_desired_world.norm();
        if (acceleration_magnitude > max_acceleration_ && acceleration_magnitude > 1e-6) {
            acceleration_desired_world = acceleration_desired_world / acceleration_magnitude * max_acceleration_;
        }  

        // Extract yaw from the NEXT pose
        tf2::Quaternion q_next(
            pose_yaw->pose.orientation.x,
            pose_yaw->pose.orientation.y,
            pose_yaw->pose.orientation.z,
            pose_yaw->pose.orientation.w);
        tf2::Matrix3x3 m_next(q_next);
        double roll_next, pitch_next;
        m_next.getRPY(roll_next, pitch_next, yaw_next);

        RCLCPP_INFO(this->get_logger(), "Yaw: %f", yaw_next);
        
        // Adjust yaw_next with symmetry handling
        yaw_next = normalizeAngle(yaw_next);
        if (std::isnan(previous_yaw)) {
            previous_yaw = yaw_next; // Initialize it with the current yaw
        }
        previous_yaw = normalizeAngle(previous_yaw);

        // Convert yaw to a 2D unit vector
        Eigen::Vector2d yaw_vector(std::cos(yaw_next), std::sin(yaw_next));

        // Compute the velocity vector in the x-y plane
        Eigen::Vector2d velocity_vector(
            velocity_desired_world.x(),
            velocity_desired_world.y()
        );
        double velocity_magnitude_xy = velocity_vector.norm();

        // Only adjust yaw if the x-y velocity magnitude is above a threshold
        if (velocity_magnitude_xy > 0.5) { // Threshold to avoid issues with small movements
            velocity_vector.normalize(); // Normalize the velocity vector

            // Compute the angle between the yaw vector and the velocity vector
            double dot_product = yaw_vector.dot(velocity_vector);
            double angle_to_velocity = std::acos(std::clamp(dot_product, -1.0, 1.0)); // Clamp to avoid numerical issues


            if (std::abs(angle_to_velocity) > max_yaw_to_velocity_angle_) {
                RCLCPP_WARN(this->get_logger(), "Yaw exceeds max allowable offset. Adjusting yaw.");
            
                // Calculate the two possible yaw values
                double yaw_positive = normalizeAngle(std::atan2(velocity_vector.y(), velocity_vector.x()) + max_yaw_to_velocity_angle_);
                double yaw_negative = normalizeAngle(std::atan2(velocity_vector.y(), velocity_vector.x()) - max_yaw_to_velocity_angle_);
            
                // Select the yaw value that is closer to the velocity vector
                if (std::abs(normalizeAngle(yaw_positive - yaw_next)) < std::abs(normalizeAngle(yaw_negative - yaw_next))) {
                    yaw_next = yaw_positive;
                } else {
                    yaw_next = yaw_negative;
                }
            }
        }

        double yaw_diff = normalizeAngle(yaw_next - previous_yaw);

        if (yaw_diff > delta_yaw) {
            yaw_next = normalizeAngle(previous_yaw + delta_yaw);
        } else if (yaw_diff < -delta_yaw) {
            yaw_next = normalizeAngle(previous_yaw - delta_yaw);
        }

        // Update the previous parameters
        previous_yaw = yaw_next;
        previous_published_velocity = velocity_desired_world;     

        // Create and publish the TrajectorySetpoint message for velocity control
        TrajectorySetpoint setpoint_msg{};
        setpoint_msg.position = {NAN, NAN, NAN};
        
        setpoint_msg.velocity = {
            static_cast<float>(velocity_desired_world.y()),
            static_cast<float>(velocity_desired_world.x()),
            static_cast<float>(-velocity_desired_world.z())
        };

        setpoint_msg.acceleration = {
            static_cast<float>(acceleration_desired_world.y()),
            static_cast<float>(acceleration_desired_world.x()),
            static_cast<float>(-acceleration_desired_world.z())
        };

        setpoint_msg.yaw = static_cast<float>(-yaw_next + M_PI / 2.0);
        setpoint_msg.yawspeed = NAN;

        publish_offboard_control_mode_velocity();
        trajectory_setpoint_publisher_->publish(setpoint_msg);
        RCLCPP_INFO(this->get_logger(), "Published Velocity Setpoint: [%f, %f, %f] m/s, Yaw: %f rad",
                    setpoint_msg.velocity[0], setpoint_msg.velocity[1], setpoint_msg.velocity[2], setpoint_msg.yaw);     

        // Arm the drone if it's not armed
        if (!armed_) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
        }
    } else if (msg->poses.size() == 1) {
        RCLCPP_WARN(this->get_logger(), "Received Path message with only one pose.");
        last_tf_pos = pos_tf; // Update the last TF position
        const auto &current_pose = msg->poses[0];
        // Calculate the velocity vector from the current pose to the tf position
        Eigen::Vector3d current_position(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        velocity_desired_world = (current_position - pos_tf) / dt; // Move to current pose
        velocity_desired_world = velocity_desired_world / velocity_desired_world.norm() * min_velocity_;

        TrajectorySetpoint setpoint_msg{};
        setpoint_msg.position = {NAN, NAN, NAN};
        
        setpoint_msg.velocity = {
            static_cast<float>(velocity_desired_world.y()),
            static_cast<float>(velocity_desired_world.x()),
            static_cast<float>(-velocity_desired_world.z())
        };

        setpoint_msg.yaw = static_cast<float>(-tf2::getYaw(tf2::Quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w)) + M_PI / 2.0);
        setpoint_msg.yawspeed = NAN;
        publish_offboard_control_mode_velocity();
        trajectory_setpoint_publisher_->publish(setpoint_msg);
    } else {
        // Use position control to hover the drone in place. Spin the drone slowly to find a valid path.
        yaw_next = previous_yaw + delta_yaw; // Make it spin around the previous yaw
        previous_yaw = yaw_next; // Update the previous yaw

        RCLCPP_WARN(this->get_logger(), "Received empty Path message");
        // Publish last_tf_pos and acceleration and current yaw
        TrajectorySetpoint setpoint_msg{};
        setpoint_msg.position = {
            static_cast<float>(last_tf_pos.y()),
            static_cast<float>(last_tf_pos.x()),
            static_cast<float>(-last_tf_pos.z())
        };
        setpoint_msg.yaw = static_cast<float>(-yaw_next+ M_PI / 2.0);
        publish_offboard_control_mode_velocity();
        trajectory_setpoint_publisher_->publish(setpoint_msg);
    }
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OffboardControl>();
    RCLCPP_INFO(node->get_logger(), "Starting Offboard Control Node (Velocity)...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}