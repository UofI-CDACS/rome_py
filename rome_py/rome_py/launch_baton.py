#!/usr/bin/python3
import rclpy
import json
from rclpy.node import Node

from rome_interfaces.msg import Baton, BatonRouteStop

class BatonPublisher(Node):
    def __init__(self):
        super().__init__('baton_publisher')

        self.declare_parameter('batons_json', '{}')

        self.published = False
        self.timer = self.create_timer(1.0, self.publish_batons)
        self.publisher_cache = {}  # cache publishers by topic to handle multiple topics

    def publish_batons(self):
        if self.published:
            return

        batons_json = self.get_parameter('batons_json').value
        try:
            batons_config = json.loads(batons_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode batons_json: {e}')
            return

        if not batons_config:
            self.get_logger().warn('No batons parameter provided.')
            return

        for baton_cfg in batons_config:
            baton = Baton()
            baton.baton_id = baton_cfg.get('baton_id', 'unknown')
            baton.owner_id = baton_cfg.get('owner_id', 'unknown')
            baton.ttl = baton_cfg.get('ttl', 0)
            baton.loop = baton_cfg.get('loop', False)
            baton.route_index = 0

            baton.route = []
            route_list = baton_cfg.get('route', [])
            for stop_cfg in route_list:
                stop = BatonRouteStop()
                stop.relay = stop_cfg.get('relay', '')
                baton.route.append(stop)

            if len(baton.route) == 0:
                self.get_logger().warn(f'Baton {baton.baton_id} has empty route, skipping.')
                continue

            first_relay = baton.route[0].relay
            topic_name = f'/{first_relay}/batons'

            if topic_name not in self.publisher_cache:
                self.publisher_cache[topic_name] = self.create_publisher(Baton, topic_name, 10)

            self.get_logger().info(f'Publishing baton {baton.baton_id} to topic {topic_name}')
            self.publisher_cache[topic_name].publish(baton)

        self.published = True
        self.timer.cancel()
        self.get_logger().info('All batons published, shutting down node.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = BatonPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

