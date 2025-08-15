from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='osep_simulation_environment',
            executable='px4_msg_converter_node',
            name='px4_msg_converter_node',
            parameters=[{
                'input_vel_cmd': '/osep/test'
            }]
        )
    ])