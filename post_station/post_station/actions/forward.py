from post_station.actions import action
from rclpy.node import Node

@action('forward')
def forward_action(station, parcel, params):
    if not isinstance(station, Node):
        raise TypeError("Expected an rclpy Node instance")

    destination = params.get('destination')
    if not destination:
        station.get_logger().error('Forward action missing "destination" parameter')
        return

    send_func = getattr(station, 'send_parcel', None)
    if not callable(send_func):
        station.get_logger().error('Station has no callable send_parcel method')
        return

    send_func(parcel, destination)
