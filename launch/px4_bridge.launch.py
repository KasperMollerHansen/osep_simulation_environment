from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='osep_simulation_environment',
            executable='px4_msg_converter_node',
            name='px4_msg_converter_node',
            parameters=[{
                'osep_vel_cmd': '/osep/vel_cmd'
            }]
        ),
        Node(
            package='osep_simulation_environment',
            executable='px4_vel_controller',
            name='px4_vel_controller',
            parameters=[{
                'path_topic': '/planner/smoothed_path',
                'osep_vel_cmd': '/osep/vel_cmd',
                'interpolation_distance': 2.0,  
                'max_speed': 25.0,
                'inspection_speed': 2.0,
                'max_yaw_to_velocity_angle_deg': 120.0,
                'frequency': 100
            }]
        )
    ])