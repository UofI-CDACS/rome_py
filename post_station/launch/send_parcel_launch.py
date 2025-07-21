import json
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import uuid

def generate_launch_description():
    #hostnames = ['rospi-1-desktop', 'rospi-2-desktop', 'rospi-3-desktop', 'rospi-4-desktop']  # simulated hostnames
    #stations = [hostname.replace('-', '_') + '/station_default' for hostname in hostnames]
    parcels = []
    parcel_count = int(LaunchConfiguration('parcel_count').perform({}))
    owner = LaunchConfiguration('owner').perform({})
    next_location = LaunchConfiguration('next_location').perform({})
    instruction_set = LaunchConfiguration('INSTRUCTION_SET').perform({})
    data = json.loads(LaunchConfiguration('data').perform({}))
    for i in range(parcel_count):
        parcels.append({
            'parcel_id': str(uuid.uuid4()),
            'owner_id': owner,
            'prev_location': 'master',
            'next_location': next_location,
            'instruction_set': instruction_set,
            'data': data
            })

    parcels_json = json.dumps(parcels)

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace to push'
        ),

        # Apply namespace to all nodes below
        PushRosNamespace(LaunchConfiguration('namespace')),

        Node(
            package='post_station',
            executable='parcel',
            parameters=[{'parcels_json': parcels_json}],
            output='screen',
        ),
    ])

#OLD
#string owner_id         # Owner of the parcel
#string parcel_id         # Unique ID per owner

#parcelRouteStop[] route  # Ordered list of stops

#uint32 ttl             # Time-to-live hops remaining
#bool loop              # Loop route if true

#uint32 route_index     # Current position in the route#

#NEW

#string owner_id            # Creator/owner of the parcel
#string parcel_id           # Unique ID for this parcel

#string prev_location       # Previous Station to hold this parcel.
#string next_location       # Targeted Station to hold this parcel.

#string instruction_set  # Which instruction set to use

#post_interfaces/KeyValue[] data  # Flexible metadata for state, ttl, error info, etc.