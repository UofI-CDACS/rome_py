import rclpy
from rclpy.node import Node
from rome_interfaces.msg import Baton, BatonRouteStop

class RelayNode(Node):
    def __init__(self):
        full_node_name = 'relay_default'
        super().__init__(full_node_name)

        topic_name = f'{full_node_name}/batons'
        self.subscription = self.create_subscription(
            Baton,
            topic_name,
            self.baton_callback,
            10
        )

        self.pubs_dict = {}

        self.get_logger().info(f'Relay node "{self.get_name()}" started, listening on topic "{topic_name}"')

    def get_publisher_for_topic(self, topic_name: str):
        if topic_name not in self.pubs_dict:
            self.pubs_dict[topic_name] = self.create_publisher(Baton, topic_name, 10)
            self.get_logger().info(f'Created publisher for topic: {topic_name}')
        return self.pubs_dict[topic_name]

    def baton_callback(self, baton: Baton):
        #self.get_logger().info(f"Received baton callback for baton_id {baton.baton_id}")

        # Check route index validity
        if baton.route_index >= len(baton.route):
            self.get_logger().warn('Baton route index out of range')
            return

        current_stop = baton.route[baton.route_index]
        this_relay = f'{self.get_namespace()}/{self.get_name()}'
        if ('/' + current_stop.relay) != this_relay:
            self.get_logger().warn(f'This baton is for {current_stop.relay}, we are {this_relay}.')
            # Not for this relay, ignore
            return

        self.get_logger().info(f'Received baton {baton.baton_id} at {current_stop.relay}')

        # Handle TTL
        if baton.ttl == 0:
            self.get_logger().info('Baton TTL expired, stopping')
            return
        baton.ttl -= 1

        # TODO: Handle other baton actions here

        # Advance route index
        baton.route_index += 1
        if baton.route_index >= len(baton.route):
            if baton.loop:
                baton.route_index = 0
            else:
                self.get_logger().info('Baton route completed')
                return

        next_relay = baton.route[baton.route_index].relay
        self.get_logger().info(f'Forwarding baton {baton.baton_id} to {next_relay}')

        next_topic = f'/{next_relay}/batons'
        publisher = self.get_publisher_for_topic(next_topic)
        publisher.publish(baton)

def main(args=None):
    rclpy.init(args=args)
    relay_node = RelayNode()
    rclpy.spin(relay_node)
    relay_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
