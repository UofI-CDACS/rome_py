from ..registry import register_action
from rclpy.node import Node
from post_interfaces.msg import KeyValue

def get_data_value(parcel, key):
    for kv in parcel.data:
        if kv.key == key:
            try:
                return int(kv.value)
            except ValueError:
                return kv.value
    return None

@register_action('check_ttl')
async def check_ttl(station: Node, parcel, key: str):
    if not isinstance(station, Node):
        raise TypeError("Expected an rclpy Node instance")
    
    if key is None:
        station.get_logger().error('check_ttl missing "key" parameter')
        return False

    value = get_data_value(parcel, key)
    if not isinstance(value, int):
        station.get_logger().error(f'check_ttl: Key "{key}" missing or not int')
        return False
    return value > 0

