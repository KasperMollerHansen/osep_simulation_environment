from launch import LaunchDescription
from launch_ros.actions import Node

TOPIC_NAMES = {
    "VEL_CMD": '/osep/vel_cmd',
    "PATH": '/osep/path',
    "COSTMAP": '/osep/local_costmap/costmap',
    "VIEWPOINTS": '/osep/viewpoints',
    "VIEWPOINTS_ADJUSTED": '/osep/viewpoints_adjusted',
    "GROUND_TRUTH": '/osep/ground_truth'
}

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='osep_simulation_environment',
            executable='px4_msg_converter_node',
            name='px4_msg_converter_node',
            parameters=[{
                'osep_vel_cmd': TOPIC_NAMES["VEL_CMD"]
            }]
        ),
        Node(
            package='osep_simulation_environment',
            executable='px4_vel_controller',
            name='px4_vel_controller',
            parameters=[{
                'path_topic': TOPIC_NAMES["PATH"],
                'osep_vel_cmd': TOPIC_NAMES["VEL_CMD"],
                'interpolation_distance': 2.0,
                'max_speed': 15.0,
                'inspection_speed': 2.0,
                'max_yaw_to_velocity_angle_deg': 120.0,
                'frequency': 100
            }]
        )
    ])