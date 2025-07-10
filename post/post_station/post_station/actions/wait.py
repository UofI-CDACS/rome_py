import asyncio
from rclpy.node import Node
from post_station.actions import action, _resolve_param

@action('wait')
async def wait_action(station, parcel, params):
    if not isinstance(station, Node):
        raise TypeError("Expected an rclpy Node instance")
    
    # Resolve seconds param (could be a direct int or a nested action)
    seconds = await _resolve_param(station, parcel, params.get('seconds', 1))
    
    try:
        seconds = float(seconds)
    except (ValueError, TypeError):
        station.get_logger().warn(f'Invalid "seconds" param for wait action, defaulting to 1')
        seconds = 1

    await asyncio.sleep(seconds)

