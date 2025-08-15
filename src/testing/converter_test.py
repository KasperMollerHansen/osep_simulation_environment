#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
import numpy as np
import math

class TestTrajectorySetpointPublisher(Node):
    def __init__(self):
        super().__init__('test_trajectory_setpoint_publisher')
        self.publisher_ = self.create_publisher(TrajectorySetpoint, '/osep/test', 10)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.count = 0
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9  # seconds
        msg = TrajectorySetpoint()

        nan = float('nan')

        if elapsed < 60.0:
            msg.position = np.array([nan, nan, nan], dtype=np.float32)
            msg.velocity = np.array([1.0, 0.0, 0.0], dtype=np.float32)
            msg.acceleration = np.array([nan, nan, nan], dtype=np.float32)
            msg.jerk = np.array([nan, nan, nan], dtype=np.float32)
            msg.yaw = nan
            msg.yawspeed = nan
            self.get_logger().info(f'Publishing constant velocity, NaN elsewhere (t={elapsed:.1f}s)')
        else:
            msg.position = np.array([0.0, 0.0, 10.0], dtype=np.float32)
            msg.velocity = np.array([nan, nan, nan], dtype=np.float32)
            msg.acceleration = np.array([nan, nan, nan], dtype=np.float32)
            msg.jerk = np.array([nan, nan, nan], dtype=np.float32)
            msg.yaw = nan
            msg.yawspeed = nan
            self.get_logger().info(f'Publishing position setpoint, NaN elsewhere (t={elapsed:.1f}s)')

        self.publisher_.publish(msg)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestTrajectorySetpointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()