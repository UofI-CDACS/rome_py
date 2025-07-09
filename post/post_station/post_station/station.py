from ament_index_python.packages import get_package_share_directory
import os
import yaml
import asyncio
import inspect
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from post_interfaces.msg import Parcel
from post_station.actions import ACTION_HANDLERS
import concurrent.futures


class Station(Node):
    def __init__(self):
        super().__init__('station_default')
        self.this_station = self.get_fully_qualified_name()
        self._instruction_sets_cache = {}
        self.subscription = self.create_subscription(
            Parcel,
            f'{self.this_station}/parcels',
            self._on_parcel_received,
            10
        )
        self._pub_cache = {}
        self.get_logger().info(f'Station "{self.this_station}" started, listening for parcels.')

    def get_instruction_set(self, set_id: str):
        if set_id in self._instruction_sets_cache:
            return self._instruction_sets_cache[set_id]

        config_dir = os.path.join(
            get_package_share_directory('post_station'),
            'config',
            'instruction_sets'
        )
        filepath = os.path.join(config_dir, f'{set_id}.yaml')

        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f) or {}
                self._instruction_sets_cache[set_id] = data
                self.get_logger().info(f'Loaded instruction set "{set_id}"')
                return data
        except FileNotFoundError:
            self.get_logger().error(f'Instruction set file not found: {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to load instruction set "{set_id}": {e}')

        return {}

    def get_publisher(self, topic_name: str):
        if topic_name not in self._pub_cache:
            self._pub_cache[topic_name] = self.create_publisher(Parcel, topic_name, 10)
            self.get_logger().info(f'Created publisher for topic: {topic_name}')
        return self._pub_cache[topic_name]
 
    def _on_parcel_received(self, parcel):
        # This lets async parcel processing run independently
        asyncio.ensure_future(self.parcel_callback(parcel))


    async def parcel_callback(self, parcel: Parcel):
        if parcel.next_destination != self.this_station:
            self.get_logger().warn(
                f'Parcel {parcel.parcel_id} not intended for this station ({self.this_station}). Ignoring.'
            )
            return

        self.get_logger().info(f'Received parcel {parcel.parcel_id} at {self.this_station}')
        instruction_set = self.get_instruction_set(parcel.instruction_set_id)
        actions = instruction_set.get(self.this_station, [])
        
        for action in actions:
            action_type = action.get('action_type')
            params = action.get('params', {})
    
            handler = ACTION_HANDLERS.get(action_type)
            if not handler:
                self.get_logger().warn(f'Unknown action: {action_type}')
                continue
    
            self.get_logger().info(f'Executing action: {action_type}')
            if inspect.iscoroutinefunction(handler):
                await handler(self, parcel, params)
            else:
                handler(self, parcel, params)

        self.get_logger().info(f'Parcel {parcel.parcel_id} processing complete.')



def main(args=None):
    rclpy.init(args=args)
    node = Station()

    async def runner():
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                await asyncio.sleep(0)  # let async tasks progress
        finally:
            node.destroy_node()
            rclpy.shutdown()

    asyncio.run(runner())

if __name__ == '__main__':
    main()
