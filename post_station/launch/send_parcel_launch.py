import json
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import uuid
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    #hostnames = ['rospi-1-desktop', 'rospi-2-desktop', 'rospi-3-desktop', 'rospi-4-desktop']  # simulated hostnames
    #stations = [hostname.replace('-', '_') + '/station_default' for hostname in hostnames]
    parcels = []
    PARCEL_COUNT = int(LaunchConfiguration('PARCEL_COUNT', default='1').perform({}))
    OWNER = LaunchConfiguration('OWNER', default='Owner').perform({})
    INSTRUCTION_SET = LaunchConfiguration('INSTRUCTION_SET', default='default').perform({})
    NEXT_LOCATION = LaunchConfiguration('NEXT_LOCATION', default='rospi_1').perform({})
    DATA = LaunchConfiguration('DATA', default='{}').perform({})
    # Generate parcels
    data = json.loads(DATA)
    for i in range(PARCEL_COUNT):
        parcels.append({
            'parcel_id': str(uuid.uuid4()),
            'owner_id': OWNER,
            'prev_location': 'master',
            'next_location': NEXT_LOCATION,
            'instruction_set': INSTRUCTION_SET,
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