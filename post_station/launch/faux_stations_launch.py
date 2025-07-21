from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='post_station',
            namespace='/faux',
            executable='station',
            name='rospi_1'
        ),
        Node(
            package='post_station',
            namespace='/faux',
            executable='station',
            name='rospi_2'
        ),
        Node(
            package='post_station',
            namespace='/faux',
            executable='station',
            name='rospi_3'
        ),
        Node(
            package='post_station',
            namespace='/faux',
            executable='station',
            name='rospi_4'
        )
    ])
