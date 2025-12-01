from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navegacao',
            executable='mapper',
            name='mapper',
            output='screen',
            parameters=[
                {'map_output_path': ''}
            ]
        )
    ])

