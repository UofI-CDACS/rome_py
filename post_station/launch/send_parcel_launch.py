import json
from launch import LaunchDescription
from launch_ros.actions import Node
import uuid

def generate_launch_description():
    #hostnames = ['rospi-1-desktop', 'rospi-2-desktop', 'rospi-3-desktop', 'rospi-4-desktop']  # simulated hostnames
    #stations = [hostname.replace('-', '_') + '/station_default' for hostname in hostnames]
    parcels = [
        {
            'parcel_id': str(uuid.uuid4()),
            'owner_id': 'master',
            'prev_location': 'master',
            'next_location': 'rospi_1',
            'instruction_set': 'loop',
            'data': [
                {'key': 'ttl', 'value': '10'},
                {'key': 'loop', 'value': 'true'}
            ]
        },
    ]

    parcels_json = json.dumps(parcels)

    return LaunchDescription([
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
