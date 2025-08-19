#pragma once
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <array>
#include <cmath>

namespace osep_simulation_environment {

class PX4MsgConverter {
public:
    static px4_msgs::msg::TrajectorySetpoint convert(const px4_msgs::msg::TrajectorySetpoint &input) {
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.timestamp = input.timestamp;
        // Coordinate transform: x=y, y=x, z=-z for all vectors
        setpoint.position[0] = input.position[1];
        setpoint.position[1] = input.position[0];
        setpoint.position[2] = -input.position[2];
        setpoint.velocity[0] = input.velocity[1];
        setpoint.velocity[1] = input.velocity[0];
        setpoint.velocity[2] = -input.velocity[2];
        setpoint.acceleration[0] = input.acceleration[1];
        setpoint.acceleration[1] = input.acceleration[0];
        setpoint.acceleration[2] = -input.acceleration[2];
        setpoint.jerk[0] = input.jerk[1];
        setpoint.jerk[1] = input.jerk[0];
        setpoint.jerk[2] = -input.jerk[2];
        setpoint.yaw = -input.yaw + static_cast<float>(M_PI / 2.0f);
        setpoint.yawspeed = -input.yawspeed;
        return setpoint;
    }
};

}