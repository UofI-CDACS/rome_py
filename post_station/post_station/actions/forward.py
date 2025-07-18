from post_station.actions import action
from rclpy.node import Node

@action('forward')
def forward(*, destination: str):
    def _run(station, parcel):
        if not isinstance(station, Node):
            raise TypeError("Expected an rclpy Node instance")
        
        if not destination:
            station.get_logger().error('Forward action missing "destination" parameter')
            return None
        
        namespace = station.get_namespace()
        if not destination.startswith('/'):
            dest = f'/{namespace.strip("/")}/{destination.strip("/")}'
        else:
            dest = destination

        parcel.prev_location = getattr(station, 'this_station', '<unknown>')
        parcel.next_location = dest

        return dest
    return _run
