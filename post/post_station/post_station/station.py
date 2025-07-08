import rclpy
from rclpy.node import Node
from post_interfaces.msg import Parcel

class Station(Node):
    def __init__(self):
        station_name = 'station_default'
        super().__init__(station_name)
        
        self.subscription = self.create_subscription(
            Parcel,
            f'{station_name}/parcels',
            self.parcel_callback,
            10
        )

        self._pub_cache = {}

        self.get_logger().info(f'Station "{station_name}" started, listening for parcels.')

    def get_publisher(self, topic_name: str):
        if topic_name not in self.publishers:
            self._pub_cache[topic_name] = self.create_publisher(Parcel, topic_name, 10)
            self.get_logger().info(f'Created publisher for topic: {topic_name}')
        return self._pub_cache[topic_name]

    def parcel_callback(self, parcel: Parcel):
        this_station = self.get_fully_qualified_name()  # includes namespace + name
        if parcel.next_destination != this_station:
            self.get_logger().warn(f'Parcel {parcel.parcel_id} not intended for this station ({this_station}). Ignoring.')
            return

        self.get_logger().info(f'Received parcel {parcel.parcel_id} at {this_station}')
        
        # TODO: Load and execute instructions based on parcel.instruction_set_id and this_station
        
        # For now, just forward to next_destination (stub logic)
        next_dest = parcel.next_destination
        
        # TEMP: prevent inf loop
        if next_dest == self.get_fully_qualified_name():
            self.get_logger().info('Already at final destination. Not forwarding.')
            return

        next_topic = f'{next_dest}/parcels'
        publisher = self.get_publisher(next_topic)
        publisher.publish(parcel)
        self.get_logger().info(f'Forwarded parcel {parcel.parcel_id} to {next_dest}')

def main(args=None):
    rclpy.init(args=args)
    node = Station()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

