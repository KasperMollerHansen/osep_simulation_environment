#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <eigen3/Eigen/Dense> // For Eigen::Vector3d
#include <limits>
#include <mutex>
#include "pid_controller.hpp"

class PX4VelController : public rclcpp::Node
{
public:
    PX4VelController()
    : Node("px4_vel_controller"),
      pid_(0.3, 0.0, 0.05), // Tune these gains!
      last_velocity_(Eigen::Vector3d::Zero())
    {
        this->declare_parameter<std::string>("path_topic", "/planner/path");
        this->declare_parameter<std::string>("osep_vel_cmd", "/osep/vel_cmd");
        this->declare_parameter<double>("interpolation_distance", 2.0);
        this->declare_parameter<double>("max_speed", 15.0);
        this->declare_parameter<double>("min_speed", 1.0);


        std::string path_topic = this->get_parameter("path_topic").as_string();
        std::string vel_cmd_topic = this->get_parameter("osep_vel_cmd").as_string();
        interpolation_distance_ = this->get_parameter("interpolation_distance").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Define QoS profile for the subscriber
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic, qos_profile,
            std::bind(&PX4VelController::path_callback, this, std::placeholders::_1)
        );

        vel_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(vel_cmd_topic, 10);

        // Timer at 1ms (1000Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // Should be 1
            std::bind(&PX4VelController::timer_callback, this)
        );
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    PIDController pid_;
    Eigen::Vector3d last_velocity_;
    Eigen::Vector3d last_acc_ = Eigen::Vector3d::Zero(); // <-- Add this line
    rclcpp::Time last_time_;

    double max_speed_{15.0}; // m/s, tune as needed
    double min_speed_{1.0};
    double interpolation_distance_{2.0};

    size_t target_idx_ = 0;

    nav_msgs::msg::Path::SharedPtr latest_path_;
    std::mutex path_mutex_;

    Eigen::Vector3d last_safe_position_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        latest_path_ = msg;
        target_idx_ = 0; // Reset target index when a new path is received
    }

    void timer_callback()
    {
        // Get latest path
        nav_msgs::msg::Path::SharedPtr path_copy;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            path_copy = latest_path_;
        }
        if (!path_copy || path_copy->poses.empty()) return;

        // Get latest transform
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Could not transform base_link to odom: %s", ex.what());
            return;
        }

        Eigen::Vector3d current_tf_pos(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z);

        // Extract yaw from quaternion
        const auto& q = transform_stamped.transform.rotation;
        tf2::Quaternion tf2_quat(q.x, q.y, q.z, q.w);
        double roll, pitch, current_tf_yaw;
        tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, current_tf_yaw);

        // Find the first pose more than interpolation_distance_ away (Ensures Stability)
        size_t i = target_idx_;
        for (; i < path_copy->poses.size(); ++i) {
            const auto &pose = path_copy->poses[i];
            Eigen::Vector3d pos(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            if ((pos - current_tf_pos).norm() > interpolation_distance_) {
                break;
            }
        }
        if (i >= path_copy->poses.size()) {
            i = path_copy->poses.size() - 1;
        }
        target_idx_ = i;

        size_t furthest_idx = target_idx_;
        Eigen::Vector3d prev_pos = current_tf_pos; // Start from current vehicle position
        double deg_to_rad = M_PI / 180.0;
        double yaw_thresh = 1.0 * deg_to_rad; // 1 degree in radians
        double collinear_dot_thresh = std::cos(yaw_thresh); // 1 degree in radians, for dot product

        // Use the yaw of the point at target_idx_ as the initial reference
        const auto& start_q = path_copy->poses[target_idx_].pose.orientation;
        tf2::Quaternion tf2_start_quat(start_q.x, start_q.y, start_q.z, start_q.w);
        double roll_start, pitch_start, prev_yaw;
        tf2::Matrix3x3(tf2_start_quat).getRPY(roll_start, pitch_start, prev_yaw); // prev_yaw is now the running reference

        for (size_t i = target_idx_; i < path_copy->poses.size(); ++i) {
            const auto &pose = path_copy->poses[i];
            Eigen::Vector3d pos(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            Eigen::Vector3d dir = pos - prev_pos;
            if (dir.norm() < 1e-6) continue; // skip if same as previous

            // For the first segment, accept it
            if (i == target_idx_) {
                furthest_idx = i;
                prev_pos = pos;
                // Extract yaw from the current pose using tf2
                const auto& q_init = pose.pose.orientation;
                tf2::Quaternion tf2_quat_init(q_init.x, q_init.y, q_init.z, q_init.w);
                double roll_init, pitch_init, yaw_init;
                tf2::Matrix3x3(tf2_quat_init).getRPY(roll_init, pitch_init, yaw_init);
                prev_yaw = yaw_init; // Set running yaw
                continue;
            }

            // Compare direction of this segment to previous segment
            Eigen::Vector3d prev_dir = prev_pos - current_tf_pos;
            if (prev_dir.norm() < 1e-6) prev_dir = dir; // fallback for very first segment
            double dot = dir.normalized().dot(prev_dir.normalized());

            // Extract yaw from pose quaternion
            const auto& q = pose.pose.orientation;
            tf2::Quaternion tf2_quat(q.x, q.y, q.z, q.w);
            double roll, pitch, pose_yaw;
            tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, pose_yaw);

            // Compare to previous accepted yaw
            double yaw_diff = std::fabs(std::atan2(std::sin(pose_yaw - prev_yaw), std::cos(pose_yaw - prev_yaw)));

            if (dot > collinear_dot_thresh && yaw_diff < yaw_thresh) {
                furthest_idx = i;
                prev_pos = pos;
                prev_yaw = pose_yaw; // <-- Update running reference
            } else {
                if (dot <= collinear_dot_thresh) {
                    RCLCPP_INFO(this->get_logger(), "Break at idx %zu: Not collinear (dot=%.3f <= thresh=%.3f)", i, dot, collinear_dot_thresh);
                }
                if (yaw_diff >= yaw_thresh) {
                    RCLCPP_INFO(this->get_logger(), "Break at idx %zu: Yaw mismatch (yaw_diff=%.3f >= thresh=%.3f)", i, yaw_diff, yaw_thresh);
                }
                break;
            }
        }
        target_idx_ = furthest_idx;
        
        const auto &target_pose = path_copy->poses[target_idx_];
        Eigen::Vector3d target_pos(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

        Eigen::Vector3d diff = target_pos - current_tf_pos;
        double distance = diff.norm();
        // Print diff
        RCLCPP_INFO(this->get_logger(), "Position Diff: [%.2f, %.2f, %.2f]",
            diff.x(), diff.y(), diff.z());

        // --- Adaptive speed based on distance ---
        double k_speed = 0.5; // Tune this gain as needed
        double adaptive_speed = std::min(max_speed_, k_speed * distance);
        adaptive_speed = std::max(min_speed_, adaptive_speed);

        Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
        if (distance > 1e-6) {
            velocity_world = diff.normalized() * adaptive_speed;
        }

       // --- PID Controller for velocity smoothing ---
        rclcpp::Time now = this->now();
        double dt = last_time_.nanoseconds() > 0 ? (now - last_time_).seconds() : 0.01;
        last_time_ = now;

        Eigen::Vector3d velocity_error = velocity_world - last_velocity_;
        Eigen::Vector3d safe_velocity = last_velocity_ + pid_.compute(velocity_error, dt);

        // Adaptive acceleration limit based on squared velocity
        double base_max_acc = 0.05;  // Minimum acceleration (gentle)
        double acc_gain = 0.05;     // Tune this gain as needed

        double speed = last_velocity_.norm();
        double max_acc = base_max_acc + acc_gain * speed * speed;

        // Optionally, clamp max_acc to a reasonable upper bound
        double max_acc_limit = 4.0; // m/s^2, tune as needed
        if (max_acc > max_acc_limit) max_acc = max_acc_limit;

        double base_max_jerk = 0.05; // m/s^3
        double jerk_gain = 0.05;     // Tune this gain as needed
        double max_jerk = base_max_jerk + jerk_gain * speed * speed;
        double max_jerk_limit = 4.0; // Clamp if needed
        if (max_jerk > max_jerk_limit) max_jerk = max_jerk_limit;

        Eigen::Vector3d acc = (safe_velocity - last_velocity_) / dt;

        // Clamp acceleration
        for (int i = 0; i < 3; ++i) {
            if (std::abs(acc[i]) > max_acc)
                acc[i] = std::copysign(max_acc, acc[i]);
        }

        // Compute jerk
        Eigen::Vector3d jerk = (acc - last_acc_) / dt;

        // Clamp jerk
        for (int i = 0; i < 3; ++i) {
            if (std::abs(jerk[i]) > max_jerk)
                acc[i] = last_acc_[i] + std::copysign(max_jerk * dt, jerk[i]);
        }

        safe_velocity = last_velocity_ + acc * dt;
        last_acc_ = acc; // Update after all clamping
        // Print velocity
        RCLCPP_INFO(this->get_logger(), "Safe Velocity: [%.2f, %.2f, %.2f]",
            safe_velocity.x(), safe_velocity.y(), safe_velocity.z());

        last_velocity_ = safe_velocity;

        // --- Yaw control ---
        // Scalar PID for yaw
        static double yaw_kp = 0.5, yaw_ki = 0.0, yaw_kd = 0.2;
        static double yaw_integral = 0.0;
        static double last_yaw_error = 0.0;
        static double last_yawspeed = 0.0;

        const auto& target_q = target_pose.pose.orientation;
        Eigen::Quaterniond target_quat(target_q.w, target_q.x, target_q.y, target_q.z);
        Eigen::Vector3d target_euler = target_quat.toRotationMatrix().eulerAngles(0, 1, 2);
        double target_yaw = target_euler[2];

        // Compute yaw error (wrap to [-pi, pi])
        double yaw_error = std::atan2(std::sin(target_yaw - current_tf_yaw), std::cos(target_yaw - current_tf_yaw));

        RCLCPP_INFO(this->get_logger(), "Current Yaw: %.3f, Target Yaw: %.3f, Yaw Error: %.3f",
            current_tf_yaw, target_yaw, yaw_error);
        
        // Precompute derivative
        double yaw_derivative = (yaw_error - last_yaw_error) / dt;

        // Detect large error jump (wraparound)
        if (std::abs(yaw_error - last_yaw_error) > M_PI) {
            yaw_integral = 0.0;
            yaw_derivative = 0.0;
            RCLCPP_WARN(this->get_logger(), "Yaw error wrap detected, resetting integral and derivative.");
        }

        yaw_integral += yaw_error * dt;
        double yawspeed_cmd = yaw_kp * yaw_error + yaw_ki * yaw_integral + yaw_kd * yaw_derivative;
        last_yaw_error = yaw_error;

        // Clamp yawspeed
        double max_yawspeed = 0.2; // rad/s, tune as needed
        yawspeed_cmd = std::clamp(yawspeed_cmd, -max_yawspeed, max_yawspeed);

        // Optionally, smooth yawspeed (rate limit)
        double max_yaw_acc = 0.05; // rad/s^2, tune as needed
        double yawspeed_acc = (yawspeed_cmd - last_yawspeed) / dt;
        if (std::abs(yawspeed_acc) > max_yaw_acc)
            yawspeed_cmd = last_yawspeed + std::copysign(max_yaw_acc * dt, yawspeed_acc);

        last_yawspeed = yawspeed_cmd;

        RCLCPP_INFO(this->get_logger(), "Yawspeed: %.3f rad/s", yawspeed_cmd);


        // Publish the safe velocity
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.position = {NAN, NAN, NAN};

        msg.velocity[0] = static_cast<float>(safe_velocity.x());
        msg.velocity[1] = static_cast<float>(safe_velocity.y());
        msg.velocity[2] = static_cast<float>(safe_velocity.z());

        msg.acceleration[0] = static_cast<float>(acc.x());
        msg.acceleration[1] = static_cast<float>(acc.y());
        msg.acceleration[2] = static_cast<float>(acc.z());


        msg.yaw = NAN;
        msg.yawspeed = yawspeed_cmd;
        vel_pub_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4VelController>());
    rclcpp::shutdown();
    return 0;
}