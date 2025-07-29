from ..registry import register_action
from rclpy.node import Node

@register_action('log_parcel')
def log_parcel(station: Node, parcel):
    if not isinstance(station, Node):
        raise TypeError("Expected an rclpy Node instance")

    parcel_id = getattr(parcel, 'parcel_id', '<unknown>')
    owner_id = getattr(parcel, 'owner_id', '<unknown>')
    prev_location = getattr(parcel, 'prev_location', '<unknown>')
    next_location = getattr(parcel, 'next_location', '<unknown>')

    msg = (
        f"[log_parcel] Parcel ID: {parcel_id}, "
        f"Owner: {owner_id}, "
        f"Prev Location: {prev_location}, "
        f"Next Location: {next_location}"
    )

    station.get_logger().info(msg)
