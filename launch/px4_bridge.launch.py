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
                'interpolation_distance': 3.0,  
                'max_speed': 15.0,
                'min_speed': 1.0,
            }]
        )
    ])