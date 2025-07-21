from ..registry import register_action
from rclpy.node import Node
from post_interfaces.msg import Parcel

@register_action('decrement_data_key')
async def decrement_data_key(station: Node, parcel: Parcel, key: str, amount: int = 1):
    for kv in parcel.data:
        if kv.key == key:
            try:
                kv.value = str(int(kv.value) - amount)
                return int(kv.value)
            except ValueError:
                raise ValueError(f'Value for key "{key}" is not a valid integer: {kv.value}')
    raise KeyError(f'Key "{key}" not found in parcel.data')
