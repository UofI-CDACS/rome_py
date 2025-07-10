from post_station.actions import action, _resolve_param
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

def set_data_value(parcel, key, value):
    for kv in parcel.data:
        if kv.key == key:
            kv.value = str(value)
            return
    parcel.data.append(KeyValue(key=key, value=str(value)))

@action('decrement_data_key')
async def decrement_data_key(station, parcel, params):
    if not isinstance(station, Node):
        raise TypeError("Expected an rclpy Node instance")
    
    key = await _resolve_param(station, parcel, params.get('key'))
    if key is None:
        station.get_logger().error('decrement_data_key missing "key" parameter')
        return False

    value = get_data_value(parcel, key)
    if not isinstance(value, int):
        station.get_logger().error(f'decrement_data_key: Key "{key}" missing or not int')
        return False

    set_data_value(parcel, key, value - 1)
    return True

