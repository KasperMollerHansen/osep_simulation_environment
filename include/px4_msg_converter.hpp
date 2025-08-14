#pragma once
#include "osep_simulation_environment/msg/action.hpp"
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <array>
#include <cmath>

namespace osep_simulation_environment {

class PX4MsgConverter {
public:
    static px4_msgs::msg::TrajectorySetpoint convert(const osep_simulation_environment::msg::Action &action) {
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.timestamp = action.timestamp;
        // Coordinate transform: x=y, y=x, z=-z for all vectors
        setpoint.position[0] = action.position[1];
        setpoint.position[1] = action.position[0];
        setpoint.position[2] = -action.position[2];
        setpoint.velocity[0] = action.velocity[1];
        setpoint.velocity[1] = action.velocity[0];
        setpoint.velocity[2] = -action.velocity[2];
        setpoint.acceleration[0] = action.acceleration[1];
        setpoint.acceleration[1] = action.acceleration[0];
        setpoint.acceleration[2] = -action.acceleration[2];
        setpoint.jerk[0] = action.jerk[1];
        setpoint.jerk[1] = action.jerk[0];
        setpoint.jerk[2] = -action.jerk[2];
        setpoint.yaw = -action.yaw + static_cast<float>(M_PI / 2.0f);
        setpoint.yawspeed = action.yawspeed;
        return setpoint;
    }
};

} // namespace osep_simulation_environment
