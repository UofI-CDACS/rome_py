import asyncio
from ..registry import register_action
from rclpy.node import Node

@register_action('wait')
async def wait_action(station: Node, parcel, seconds: float = 1.0):
    if not isinstance(station, Node):
        raise TypeError("Expected an rclpy Node instance")

    try:
        seconds = float(seconds)
    except (ValueError, TypeError):
        station.get_logger().warn(f'Invalid "seconds" param for wait action, defaulting to 1')
        seconds = 1

    await asyncio.sleep(seconds)
