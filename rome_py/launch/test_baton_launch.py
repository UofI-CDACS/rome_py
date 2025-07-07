import json
from launch import LaunchDescription
from launch_ros.actions import Node
import uuid

def generate_launch_description():
    hostnames = ['rospi-1-desktop', 'rospi-2-desktop', 'rospi-3-desktop', 'rospi-4-desktop']  # simulated hostnames
    relays = [hostname.replace('-', '_') + '/relay_default' for hostname in hostnames]
    batons = [
        {
            'baton_id': str(uuid.uuid4()),
            'owner_id': 'master',
            'ttl': 10,
            'loop': False,
            'route': [{'relay': relays[0]}, {'relay': relays[1]}, {'relay': relays[2]}]
        },
        {
            'baton_id': str(uuid.uuid4()),
            'owner_id': 'operator',
            'ttl': 5,
            'loop': True,
            'route': [{'relay': relays[2]}, {'relay': relays[3]}]
        },
    ]

    batons_json = json.dumps(batons)

    return LaunchDescription([
        Node(
            package='rome_py',
            executable='launch_baton',
            parameters=[{'batons_json': batons_json}],
            output='screen',
        ),
    ])

