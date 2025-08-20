#include "px4_vel_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <limits>
#include <cmath>
#include <algorithm>

PX4VelController::PX4VelController()
: Node("px4_vel_controller"),
  vel_pid_(0.3, 0.0, 0.2),
  yaw_pid_(2.0, 0.0, 0.5),
  last_velocity_(Eigen::Vector3d::Zero()),
  last_acc_(Eigen::Vector3d::Zero()),
  target_idx_(0)
{
    // Tunable parameters
    this->declare_parameter<std::string>("path_topic", "/planner/path");
    this->declare_parameter<std::string>("osep_vel_cmd", "/osep/vel_cmd");
    this->declare_parameter<double>("interpolation_distance", 2.0);
    this->declare_parameter<double>("max_speed", 15.0);
    this->declare_parameter<double>("inspection_speed", 1.0);
    this->declare_parameter<double>("max_yaw_to_velocity_angle_deg", 120.0);
    this->declare_parameter<int>("frequency", 1000);

    std::string path_topic = this->get_parameter("path_topic").as_string();
    std::string vel_cmd_topic = this->get_parameter("osep_vel_cmd").as_string();
    interpolation_distance_ = this->get_parameter("interpolation_distance").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    inspection_speed_ = this->get_parameter("inspection_speed").as_double();
    double max_yaw_to_velocity_angle_deg = this->get_parameter("max_yaw_to_velocity_angle_deg").as_double();
    max_yaw_to_velocity_angle_ = max_yaw_to_velocity_angle_deg * M_PI / 180.0;
    int frequency = this->get_parameter("frequency").as_int();
    int ms_per_cycle = 1000 / frequency;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic, qos_profile,
        std::bind(&PX4VelController::path_callback, this, std::placeholders::_1)
    );

    vel_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(vel_cmd_topic, 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(ms_per_cycle),
        std::bind(&PX4VelController::timer_callback, this)
    );

    last_path_time_ = this->now();
}

void PX4VelController::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_path_ = msg;
    target_idx_ = 0;
    last_path_time_ = this->now();
}

bool PX4VelController::get_current_pose(Eigen::Vector3d &pos, double &yaw)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Could not transform base_link to odom: %s", ex.what());
        return false;
    }
    pos = Eigen::Vector3d(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z);
    const auto& q = transform_stamped.transform.rotation;
    tf2::Quaternion tf2_quat(q.x, q.y, q.z, q.w);
    double roll, pitch;
    tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
    return true;
}

size_t PX4VelController::find_target_idx(const nav_msgs::msg::Path::SharedPtr &path, const Eigen::Vector3d &current_pos)
{
    size_t i = target_idx_;
    for (; i < path->poses.size(); ++i) {
        const auto &pose = path->poses[i];
        Eigen::Vector3d pos(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        if ((pos - current_pos).norm() > interpolation_distance_) {
            break;
        }
    }
    if (i >= path->poses.size()) i = path->poses.size() - 1;
    return i;
}

size_t PX4VelController::find_furthest_collinear_idx(const nav_msgs::msg::Path::SharedPtr &path, size_t start_idx, const Eigen::Vector3d &current_pos, double &ref_yaw)
{
    // Tunable parameters
    double yaw_thresh = 2.0 * M_PI / 180.0;
    double collinear_dot_thresh = std::cos(yaw_thresh);

    size_t furthest_idx = start_idx;
    Eigen::Vector3d prev_pos = current_pos;

    // Use the yaw of the point at start_idx as the initial reference
    const auto& start_q = path->poses[start_idx].pose.orientation;
    tf2::Quaternion tf2_start_quat(start_q.x, start_q.y, start_q.z, start_q.w);
    double roll_start, pitch_start;
    tf2::Matrix3x3(tf2_start_quat).getRPY(roll_start, pitch_start, ref_yaw);

    for (size_t i = start_idx; i < path->poses.size(); ++i) {
        const auto &pose = path->poses[i];
        Eigen::Vector3d pos(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        Eigen::Vector3d dir = pos - prev_pos;
        if (dir.norm() < 1e-6) continue;
        if (i == start_idx) {
            furthest_idx = i;
            prev_pos = pos;
            const auto& q_init = pose.pose.orientation;
            tf2::Quaternion tf2_quat_init(q_init.x, q_init.y, q_init.z, q_init.w);
            double roll_init, pitch_init, yaw_init;
            tf2::Matrix3x3(tf2_quat_init).getRPY(roll_init, pitch_init, yaw_init);
            ref_yaw = yaw_init;
            continue;
        }
        Eigen::Vector3d prev_dir = prev_pos - current_pos;
        if (prev_dir.norm() < 1e-6) prev_dir = dir;
        double dot = dir.normalized().dot(prev_dir.normalized());
        const auto& q = pose.pose.orientation;
        tf2::Quaternion tf2_quat(q.x, q.y, q.z, q.w);
        double roll, pitch, pose_yaw;
        tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, pose_yaw);
        double yaw_diff = std::fabs(std::atan2(std::sin(pose_yaw - ref_yaw), std::cos(pose_yaw - ref_yaw)));
        if (dot > collinear_dot_thresh && yaw_diff < yaw_thresh) {
            furthest_idx = i;
            prev_pos = pos;
            ref_yaw = pose_yaw;
        } else {
            break;
        }
    }
    return furthest_idx;
}

double PX4VelController::compute_adaptive_speed(double distance, double effective_angle)
{
    // Tunable parameters
    double k_speed = 0.15;
    double sharp_turn_thresh = M_PI / 3.0; // 60 degrees

    double adaptive_speed = 0.0;
    if (distance > 1e-6) {
        adaptive_speed = k_speed * std::pow(distance, 1.3);
        adaptive_speed = std::min(max_speed_, adaptive_speed);
        if (effective_angle < sharp_turn_thresh) {
            adaptive_speed = std::max(inspection_speed_, adaptive_speed);
        }
    }
    return adaptive_speed;
}

Eigen::Vector3d PX4VelController::compute_safe_velocity(const Eigen::Vector3d &desired_velocity, double dt)
{
    // Tunable parameters
    double base_max_acc = 0.1;
    double acc_gain = 0.04;
    double max_acc_limit = 4.0;
    double base_max_jerk = 0.1;
    double jerk_gain = 0.04;
    double max_jerk_limit = 4.0;

    // Velocity PID
    Eigen::Vector3d velocity_error = desired_velocity - last_velocity_;
    Eigen::Vector3d safe_velocity = last_velocity_ + vel_pid_.compute(velocity_error, dt);

    double speed = last_velocity_.norm();
    double max_acc = base_max_acc + acc_gain * speed * speed;
    if (max_acc > max_acc_limit) max_acc = max_acc_limit;

    double max_jerk = base_max_jerk + jerk_gain * speed * speed;
    if (max_jerk > max_jerk_limit) max_jerk = max_jerk_limit;

    Eigen::Vector3d acc = (safe_velocity - last_velocity_) / dt;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(acc[i]) > max_acc)
            acc[i] = std::copysign(max_acc, acc[i]);
    }

    // Jerk limiting
    Eigen::Vector3d jerk = (acc - last_acc_) / dt;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(jerk[i]) > max_jerk)
            acc[i] = last_acc_[i] + std::copysign(max_jerk * dt, jerk[i]);
    }

    safe_velocity = last_velocity_ + acc * dt;

    last_acc_ = acc;
    last_velocity_ = safe_velocity;
    return safe_velocity;
}

double PX4VelController::clamp_angle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double PX4VelController::compute_yawspeed(double target_yaw, double current_yaw, double dt)
{
    double max_yawspeed = 0.25;
    double max_yaw_acc = 10.0;

    static double last_yawspeed = 0.0;
    double yaw_error = std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw));

    // Reset PID if error wraps
    static double last_yaw_error = 0.0;
    if (std::abs(yaw_error - last_yaw_error) > M_PI) {
        yaw_pid_.reset();
        RCLCPP_WARN(this->get_logger(), "Yaw error wrap detected, resetting PID.");
    }
    last_yaw_error = yaw_error;

    double yawspeed_cmd = yaw_pid_.compute(yaw_error, dt);
    yawspeed_cmd = std::clamp(yawspeed_cmd, -max_yawspeed, max_yawspeed);

    double yawspeed_acc = (yawspeed_cmd - last_yawspeed) / dt;
    if (std::abs(yawspeed_acc) > max_yaw_acc)
        yawspeed_cmd = last_yawspeed + std::copysign(max_yaw_acc * dt, yawspeed_acc);
    last_yawspeed = yawspeed_cmd;
    return yawspeed_cmd;
}

bool PX4VelController::is_path_timeout() {
    const double path_timeout_sec = 5.0;
    return (this->now() - last_path_time_).seconds() > path_timeout_sec;
}

bool PX4VelController::get_valid_path(nav_msgs::msg::Path::SharedPtr& path_copy) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_copy = latest_path_;
    return path_copy && !path_copy->poses.empty();
}

bool PX4VelController::get_valid_pose(Eigen::Vector3d& pos, double& yaw) {
    return get_current_pose(pos, yaw);
}

void PX4VelController::update_target_indices(const nav_msgs::msg::Path::SharedPtr& path, const Eigen::Vector3d& current_tf_pos) {
    target_idx_ = find_target_idx(path, current_tf_pos);
    double ref_yaw = 0.0;
    target_idx_ = find_furthest_collinear_idx(path, target_idx_, current_tf_pos, ref_yaw);
}

Eigen::Vector3d PX4VelController::calculate_safe_velocity(
    const nav_msgs::msg::Path::SharedPtr& path,
    const Eigen::Vector3d& current_tf_pos,
    double& effective_angle)
{
    const int lookahead_points = 5;
    const auto &target_pose = path->poses[target_idx_];
    Eigen::Vector3d target_pos(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    Eigen::Vector3d diff = target_pos - current_tf_pos;
    double distance = diff.norm();
    RCLCPP_INFO(this->get_logger(), "Position Diff: [%.2f, %.2f, %.2f]", diff.x(), diff.y(), diff.z());

    Eigen::Vector3d desired_dir = diff.normalized();
    double current_speed = last_velocity_.norm();
    Eigen::Vector3d current_dir = (current_speed > 1e-3) ? last_velocity_.normalized() : desired_dir;
    double dir_dot = desired_dir.dot(current_dir);
    dir_dot = std::clamp(dir_dot, -1.0, 1.0);
    double angle = std::acos(dir_dot);

    Eigen::Vector3d avg_look_dir = Eigen::Vector3d::Zero();
    int avg_count = 0;
    for (int k = 1; k <= lookahead_points; ++k) {
        if (target_idx_ + k < path->poses.size()) {
            const auto& next_pose = path->poses[target_idx_ + k];
            Eigen::Vector3d next_pos(next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z);
            Eigen::Vector3d look_dir = (next_pos - target_pos);
            if (look_dir.norm() < 1e-6) continue;
            avg_look_dir += look_dir.normalized();
            avg_count++;
        }
    }
    effective_angle = angle;
    if (avg_count > 0) {
        avg_look_dir.normalize();
        double avg_dot = desired_dir.dot(avg_look_dir);
        avg_dot = std::clamp(avg_dot, -1.0, 1.0);
        double avg_lookahead_angle = std::acos(avg_dot);
        effective_angle = std::max(angle, avg_lookahead_angle);
    }

    double adaptive_speed = compute_adaptive_speed(distance, effective_angle);
    Eigen::Vector3d velocity_world = desired_dir * adaptive_speed;

    rclcpp::Time now = this->now();
    double dt = last_time_.nanoseconds() > 0 ? (now - last_time_).seconds() : 0.01;
    last_time_ = now;

    Eigen::Vector3d safe_velocity = compute_safe_velocity(velocity_world, dt);

    // --- CAP SAFE_VELOCITY BASED ON PATH DIRECTION ---
    if (distance > 10 * interpolation_distance_) {
        // Project safe_velocity onto desired_dir
        double along_path = safe_velocity.dot(desired_dir);
        // Clamp to [-adaptive_speed, adaptive_speed]
        along_path = std::clamp(along_path, -adaptive_speed, adaptive_speed);
        // Remove perpendicular component if you want strict following:
        safe_velocity = desired_dir * along_path;
    }

    RCLCPP_INFO(this->get_logger(), "Safe Velocity: [%.2f, %.2f, %.2f]", safe_velocity.x(), safe_velocity.y(), safe_velocity.z());
    return safe_velocity;
}

void PX4VelController::calculate_yaw(
    const nav_msgs::msg::Path::SharedPtr& path,
    const Eigen::Vector3d& safe_velocity,
    const Eigen::Vector3d& current_tf_pos,
    double& target_yaw,
    double& yawspeed_cmd,
    double current_tf_yaw)
{
    const auto &target_pose = path->poses[target_idx_];
    const auto& target_q = target_pose.pose.orientation;
    tf2::Quaternion tf2_target_quat(target_q.x, target_q.y, target_q.z, target_q.w);
    double roll_target, pitch_target;
    tf2::Matrix3x3(tf2_target_quat).getRPY(roll_target, pitch_target, target_yaw);

    Eigen::Vector2d velocity_xy(safe_velocity.x(), safe_velocity.y());
    double velocity_magnitude_xy = velocity_xy.norm();

    if (velocity_magnitude_xy > interpolation_distance_/4.0) {
        Eigen::Vector2d velocity_vector = velocity_xy.normalized();
        Eigen::Vector2d yaw_vector(std::cos(target_yaw), std::sin(target_yaw));
        double dot_product = yaw_vector.dot(velocity_vector);
        double angle_to_velocity = std::acos(std::clamp(dot_product, -1.0, 1.0));
        if (std::abs(angle_to_velocity) > max_yaw_to_velocity_angle_) {
            RCLCPP_WARN(this->get_logger(), "Yaw exceeds max allowable offset. Adjusting yaw.");
            double velocity_angle = std::atan2(velocity_vector.y(), velocity_vector.x());
            double yaw_positive = clamp_angle(velocity_angle + max_yaw_to_velocity_angle_);
            double yaw_negative = clamp_angle(velocity_angle - max_yaw_to_velocity_angle_);
            if (std::abs(clamp_angle(yaw_positive - target_yaw)) < std::abs(clamp_angle(yaw_negative - target_yaw))) {
                target_yaw = yaw_positive;
            } else {
                target_yaw = yaw_negative;
            }
        }
    }

    double dt = last_time_.nanoseconds() > 0 ? (this->now() - last_time_).seconds() : 0.01;
    yawspeed_cmd = compute_yawspeed(target_yaw, current_tf_yaw, dt);

    RCLCPP_INFO(this->get_logger(), "Current Yaw: %.3f, Target Yaw: %.3f, Yaw Error: %.3f",
        current_tf_yaw, target_yaw, std::atan2(std::sin(target_yaw - current_tf_yaw), std::cos(target_yaw - current_tf_yaw)));
    RCLCPP_INFO(this->get_logger(), "Yawspeed: %.3f rad/s", yawspeed_cmd);
}

void PX4VelController::publish_vel_setpoint(const Eigen::Vector3d& safe_velocity, double yawspeed_cmd)
{
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.position = {NAN, NAN, NAN};
    msg.velocity[0] = static_cast<float>(safe_velocity.x());
    msg.velocity[1] = static_cast<float>(safe_velocity.y());
    msg.velocity[2] = static_cast<float>(safe_velocity.z());
    // msg.acceleration[0] = static_cast<float>(last_acc_.x());
    // msg.acceleration[1] = static_cast<float>(last_acc_.y());
    // msg.acceleration[2] = static_cast<float>(last_acc_.z());

    msg.yaw = NAN;
    msg.yawspeed = static_cast<float>(yawspeed_cmd);
    vel_pub_->publish(msg);
}

void PX4VelController::publish_safe_setpoint(const Eigen::Vector3d& safe_setpoint, double yaw_cmd)
{
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.position[0] = static_cast<float>(safe_setpoint.x());
    msg.position[1] = static_cast<float>(safe_setpoint.y());
    msg.position[2] = static_cast<float>(safe_setpoint.z());

    msg.yaw = static_cast<float>(yaw_cmd);
    vel_pub_->publish(msg);
}

void PX4VelController::timer_callback()
{
    static bool has_valid_pose = false;
    Eigen::Vector3d current_tf_pos;
    double current_tf_yaw;

    nav_msgs::msg::Path::SharedPtr path_copy;
    bool pose_ok = get_valid_pose(current_tf_pos, current_tf_yaw);
    bool path_ok = !is_path_timeout() && get_valid_path(path_copy);

    if (!pose_ok || !path_ok) {
        if (has_valid_pose) {
            publish_safe_setpoint(last_valid_tf_pos_, last_valid_tf_yaw_);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "No valid pose or path (timeout or empty), publishing last valid position and yaw.");
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "No valid pose received yet, skipping control output.");
        }
        return;
    }

    // Update last valid pose/yaw only when both are valid
    last_valid_tf_pos_ = current_tf_pos;
    last_valid_tf_yaw_ = current_tf_yaw;
    has_valid_pose = true;

    update_target_indices(path_copy, current_tf_pos);

    double effective_angle = 0.0;
    Eigen::Vector3d safe_velocity = calculate_safe_velocity(path_copy, current_tf_pos, effective_angle);

    double target_yaw = 0.0;
    double yawspeed_cmd = 0.0;
    calculate_yaw(path_copy, safe_velocity, current_tf_pos, target_yaw, yawspeed_cmd, current_tf_yaw);

    publish_vel_setpoint(safe_velocity, yawspeed_cmd);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4VelController>());
    rclcpp::shutdown();
    return 0;
}