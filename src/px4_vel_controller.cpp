#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
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
      pid_(1.0, 0.0, 0.1), // Tune these gains!
      last_velocity_(Eigen::Vector3d::Zero())
    {
        this->declare_parameter<std::string>("path_topic", "/planner/path");
        this->declare_parameter<std::string>("osep_vel_cmd", "/osep/vel_cmd");
        this->declare_parameter<double>("interpolation_distance", 2.0);
        this->declare_parameter<double>("max_speed", 15.0);


        std::string path_topic = this->get_parameter("path_topic").as_string();
        std::string vel_cmd_topic = this->get_parameter("osep_vel_cmd").as_string();
        interpolation_distance_ = this->get_parameter("interpolation_distance").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();

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

        // Find the first pose more than interpolation_distance_ away
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
        Eigen::Vector3d start = current_tf_pos;
        Eigen::Vector3d ref_dir = (Eigen::Vector3d(
            path_copy->poses.back().pose.position.x,
            path_copy->poses.back().pose.position.y,
            path_copy->poses.back().pose.position.z
        ) - start).normalized();

        double collinear_thresh = 0.5; // This needs tuning!!
        for (size_t i = target_idx_; i < path_copy->poses.size(); ++i) {
            const auto &pose = path_copy->poses[i];
            Eigen::Vector3d pos(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            Eigen::Vector3d dir = pos - start;
            if (dir.norm() < 1e-6) continue; // skip if same as start

            // Check if direction is collinear with reference direction
            double cross_norm = (dir.normalized().cross(ref_dir)).norm();
            if (cross_norm < collinear_thresh) {
                furthest_idx = i;
            } else {
                break; // Stop at first non-collinear point
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
        double k_speed = 1.0; // Tune this gain as needed
        double adaptive_speed = std::min(max_speed_, k_speed * distance);

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

        // Limit acceleration and jerk
        double max_acc = 1.0; // m/s^2, tune as needed
        double max_jerk = 2.0; // m/s^3, tune as needed

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


        // Publish the safe velocity
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.position = {NAN, NAN, NAN};

        msg.velocity[0] = static_cast<float>(safe_velocity.x());
        msg.velocity[1] = static_cast<float>(safe_velocity.y());
        msg.velocity[2] = static_cast<float>(safe_velocity.z());
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