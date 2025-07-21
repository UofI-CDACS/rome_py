from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ns = LaunchConfiguration('namespace')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='faux',
        description='Namespace under which all stations will be launched'
    )

    station_nodes = []
    for i in range(1, 5):
        station_nodes.append(
            Node(
                package='post_core',
                executable='station',
                namespace=ns,
                name=f'rospi_{i}',
                parameters=[],
                output='screen',
                arguments=['--name', f'rospi_{i}', '--type', 'default'],
            )
        )

    return LaunchDescription([
        declare_namespace,
        *station_nodes
    ])

