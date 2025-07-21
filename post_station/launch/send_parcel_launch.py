import json
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import uuid

def generate_parcels_node(context):
    parcel_count = int(LaunchConfiguration('PARCEL_COUNT').perform(context))
    owner = LaunchConfiguration('OWNER').perform(context)
    instruction_set = LaunchConfiguration('INSTRUCTION_SET').perform(context)
    next_location = LaunchConfiguration('NEXT_LOCATION').perform(context)
    data = json.loads(LaunchConfiguration('DATA').perform(context))

    parcels = []
    for _ in range(parcel_count):
        parcels.append({
            'parcel_id': str(uuid.uuid4()),
            'owner_id': owner,
            'prev_location': 'master',
            'next_location': next_location,
            'instruction_set': instruction_set,
            'data': data
        })

    parcels_json = json.dumps(parcels)
    
    return [
        Node(
            package='post_station',
            executable='parcel',
            parameters=[{'parcels_json': parcels_json}],
            output='screen',
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('PARCEL_COUNT', default_value='1'),
        DeclareLaunchArgument('OWNER', default_value='Owner'),
        DeclareLaunchArgument('INSTRUCTION_SET', default_value='default'),
        DeclareLaunchArgument('NEXT_LOCATION', default_value='rospi_1'),
        DeclareLaunchArgument('DATA', default_value='{}'),
        DeclareLaunchArgument('namespace', default_value=''),
        
        PushRosNamespace(LaunchConfiguration('namespace')),
        
        # Use OpaqueFunction to generate the node with proper context
        OpaqueFunction(function=generate_parcels_node),
    ])