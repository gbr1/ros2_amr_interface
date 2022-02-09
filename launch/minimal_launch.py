from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_amr_interface',
            executable='amr_interface_node',
            name='amr',
            remappings=[
                ('/amr/cmd_vel', '/cmd_vel')
            ],
            parameters=[{
                'try_reconnect': False
                }]
        )
    ])