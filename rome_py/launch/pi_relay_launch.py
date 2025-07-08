import socket
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    hostname = socket.gethostname() 
    relay_node = [
        Node(
            package='rome_py',
            executable='launch_relay',
            namespace=hostname.replace('-', '_'),  # replace dashes with underscores here
            # name='relay_default',  # optionally specify node name
        )
    ]

    return LaunchDescription(relay_node)

