#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include "pid_controller.hpp"

class PX4VelController : public rclcpp::Node
{
public:
    PX4VelController();

private:
    // ROS
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    PIDController pid_;
    Eigen::Vector3d last_velocity_;
    Eigen::Vector3d last_acc_;
    rclcpp::Time last_time_;
    double max_speed_;
    double inspection_speed_;
    double interpolation_distance_;
    double max_yaw_to_velocity_angle_;
    size_t target_idx_;
    nav_msgs::msg::Path::SharedPtr latest_path_;
    std::mutex path_mutex_;
    Eigen::Vector3d last_safe_position_;

    // Callbacks
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void timer_callback();

    // Helper functions
    bool get_current_pose(Eigen::Vector3d &pos, double &yaw);
    size_t find_target_idx(const nav_msgs::msg::Path::SharedPtr &path, const Eigen::Vector3d &current_pos);
    size_t find_furthest_collinear_idx(const nav_msgs::msg::Path::SharedPtr &path, size_t start_idx, const Eigen::Vector3d &current_pos, double &ref_yaw);
    double compute_adaptive_speed(double distance, double effective_angle);
    Eigen::Vector3d compute_safe_velocity(const Eigen::Vector3d &desired_velocity, double dt);
    double compute_yawspeed(double target_yaw, double current_yaw, double dt);
    double clamp_angle(double angle);
};