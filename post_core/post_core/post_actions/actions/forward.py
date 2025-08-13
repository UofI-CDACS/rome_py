from ..registry import register_action
from rclpy.node import Node
from post_interfaces.msg import Parcel

@register_action('forward')
async def forward(station: Node, parcel: Parcel, destination: str):
    if not destination:
        station.get_logger().error('Forward action missing "destination" parameter')
        return None

    namespace = station.get_namespace().strip("/")
    if not destination.startswith('/'):
        destination = f'/{namespace}/{destination.strip("/")}' if namespace else f'/{destination.strip("/")}'

    return destination
