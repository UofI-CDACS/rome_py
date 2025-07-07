from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    hostnames = ['rospi-1-desktop', 'rospi-2-desktop', 'rospi-3-desktop', 'rospi-4-desktop']  # simulated hostnames

    relay_nodes = [
        Node(
            package='rome_py',
            executable='launch_relay',
            namespace=hostname.replace('-', '_'),  # replace dashes with underscores here
            # name='relay_default',  # optionally specify node name
        )
        for hostname in hostnames
    ]

    return LaunchDescription(relay_nodes)

