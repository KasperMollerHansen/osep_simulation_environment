#pragma once
#include <eigen3/Eigen/Dense>

class PIDController {
public:
    // Scalar constructor
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd),
          scalar_integral_(0.0), scalar_last_error_(0.0),
          vector_integral_(Eigen::Vector3d::Zero()), vector_last_error_(Eigen::Vector3d::Zero()),
          is_vector_(false) {}

    // Vector constructor
    PIDController(double kp, double ki, double kd, bool is_vector)
        : kp_(kp), ki_(ki), kd_(kd),
          scalar_integral_(0.0), scalar_last_error_(0.0),
          vector_integral_(Eigen::Vector3d::Zero()), vector_last_error_(Eigen::Vector3d::Zero()),
          is_vector_(is_vector) {}

    // Scalar compute
    double compute(double error, double dt) {
        scalar_integral_ += error * dt;
        double derivative = (error - scalar_last_error_) / dt;
        scalar_last_error_ = error;
        return kp_ * error + ki_ * scalar_integral_ + kd_ * derivative;
    }

    // Vector compute
    Eigen::Vector3d compute(const Eigen::Vector3d& error, double dt) {
        vector_integral_ += error * dt;
        Eigen::Vector3d derivative = (error - vector_last_error_) / dt;
        vector_last_error_ = error;
        return kp_ * error + ki_ * vector_integral_ + kd_ * derivative;
    }

    void reset() {
        scalar_integral_ = 0.0;
        scalar_last_error_ = 0.0;
        vector_integral_.setZero();
        vector_last_error_.setZero();
    }

private:
    double kp_, ki_, kd_;
    // Scalar state
    double scalar_integral_;
    double scalar_last_error_;
    // Vector state
    Eigen::Vector3d vector_integral_;
    Eigen::Vector3d vector_last_error_;
    bool is_vector_;
};