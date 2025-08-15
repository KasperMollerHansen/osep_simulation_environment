from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='osep_simulation_environment',
            executable='px4_bridge',
            name='px4_bridge',
            parameters=[{
                'input_vel_cmd': '/osep/test'  # You can override this at launch
            }]
        )
    ])