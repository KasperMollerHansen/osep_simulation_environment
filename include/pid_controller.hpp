#pragma once
#include <eigen3/Eigen/Dense>

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(Eigen::Vector3d::Zero()), prev_error_(Eigen::Vector3d::Zero()) {}

    Eigen::Vector3d compute(const Eigen::Vector3d& error, double dt) {
        integral_ += error * dt;
        Eigen::Vector3d derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        integral_.setZero();
        prev_error_.setZero();
    }

private:
    double kp_, ki_, kd_;
    Eigen::Vector3d integral_;
    Eigen::Vector3d prev_error_;
};