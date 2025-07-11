#!/usr/bin/python3
import rclpy
import json
from rclpy.node import Node

from post_interfaces.msg import Parcel, KeyValue

class ParcelPublisher(Node):
    def __init__(self):
        super().__init__('parcel_publisher')

        self.declare_parameter('parcels_json', '{}')

        self.published = False
        self.timer = self.create_timer(1.0, self.publish_parcels)
        self.publisher_cache = {}  # cache publishers by topic to handle multiple topics

    def publish_parcels(self):
        if self.published:
            return

        parcels_json = self.get_parameter('parcels_json').value
        try:
            parcels_config = json.loads(parcels_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode parcels_json: {e}')
            return

        if not parcels_config:
            self.get_logger().warn('No parcels parameter provided.')
            return

        for parcel_cfg in parcels_config:
            parcel = Parcel()
            parcel.parcel_id = parcel_cfg.get('parcel_id', 'unknown')
            parcel.owner_id = parcel_cfg.get('owner_id', 'unknown')
            parcel.prev_location = parcel_cfg.get('prev_location', '')
            parcel.next_location = '/' + parcel_cfg.get('next_location', '')
            parcel.instruction_set = parcel_cfg.get('instruction_set', '')
            # Fill flexible metadata
            parcel.data = []
            for kv_cfg in parcel_cfg.get('data', []):
                kv = KeyValue()
                kv.key = kv_cfg.get('key', '')
                kv.value = kv_cfg.get('value', '')
                parcel.data.append(kv)
            # Use next_location for topic name
            topic_name = f'/{parcel.next_location}/parcels'

            if topic_name not in self.publisher_cache:
                self.publisher_cache[topic_name] = self.create_publisher(Parcel, topic_name, 10)

            self.get_logger().info(f'Publishing parcel {parcel.parcel_id} to topic {topic_name}')
            self.publisher_cache[topic_name].publish(parcel)

            self.published = True
            self.timer.cancel()
            self.get_logger().info('All parcels published, shutting down node.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ParcelPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
