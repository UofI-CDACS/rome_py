from post_actions import action
from rclpy.node import Node

@action
async def forward(station: Node, parcel: Any, destination: str):
    if not destination:
        station.get_logger().error('Forward action missing "destination" parameter')
        return None

    namespace = station.get_namespace()
    if not destination.startswith('/'):
        destination = f'/{namespace.strip("/")}/{destination.strip("/")}'

    parcel.prev_location = getattr(station, 'this_station', '<unknown>')
    parcel.next_location = destination

    return destination

