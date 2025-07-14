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
from post_station.actions import GRAVEYARD_SIGNAL
import concurrent.futures


class Station(Node):
    def __init__(self):
        super().__init__('station_default')
        self.this_station = self.get_fully_qualified_name()
        print(self.get_fully_qualified_name())
        
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
                graveyard = data.get('graveyard', 'default_graveyard')
                self._instruction_sets_cache[set_id] = {
                    'actions': data,
                    'graveyard': graveyard
                }
                return self._instruction_sets_cache[set_id]
        except FileNotFoundError:
            self.get_logger().error(f'Instruction set file not found: {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to load instruction set "{set_id}": {e}')

        return {'actions': {}, 'graveyard': 'default_graveyard'}

    def get_publisher(self, topic_name: str):
        if topic_name not in self._pub_cache:
            self._pub_cache[topic_name] = self.create_publisher(Parcel, topic_name, 10)
        return self._pub_cache[topic_name]

    def send_parcel(self, parcel, next_location: str):
        # Update parcel locations
        parcel.prev_location = self.this_station
        parcel.next_location = next_location 
    
        # Prepare topic and publisher
        topic = f'{next_location}/parcels'
        publisher = self.get_publisher(topic)
        
        # Publish parcel
        publisher.publish(parcel)

    def _on_parcel_received(self, parcel):
        # self.get_logger().info(f'Received parcel {parcel.parcel_id} destined for {parcel.next_location}')
        # This lets async parcel processing run independently
        asyncio.ensure_future(self.parcel_callback(parcel))

    async def parcel_callback(self, parcel: Parcel):
        if parcel.next_location != self.this_station:
            self.get_logger().warn(
                f'Parcel {parcel.parcel_id} not intended for this station ({self.this_station}). Ignoring.'
            )
            return
        
        instruction_set_data = self.get_instruction_set(parcel.instruction_set)
        actions_map = instruction_set_data.get('actions', {})

        actions_map = instruction_set_data.get('actions', {})
        
        actions = []
        
        # Case 1: fully qualified match if key is prefixed with '/'
        if any(k.startswith('/') for k in actions_map):
            actions = actions_map.get(self.this_station, [])
            #self.get_logger().info(f'Checked full name "{self.this_station}" → {len(actions)} actions.')
        
        # Case 2: short name match only if no slash prefix
        else:
            station_tail = self.this_station.strip('/').split('/')[-1]
            actions = actions_map.get(station_tail, [])
            #self.get_logger().info(f'Checked short name "{station_tail}" → {len(actions)} actions.')
        
        graveyard = instruction_set_data.get('graveyard')
        
        for action in actions:
            action_type = action.get('action_type')
            params = action.get('params', {})

            handler = ACTION_HANDLERS.get(action_type)
            if not handler:
                self.get_logger().warn(f'Unknown action: {action_type}')
                result = GRAVEYARD_SIGNAL
            else:
                try:
                    if inspect.iscoroutinefunction(handler):
                        result = await handler(self, parcel, params)
                    else:
                        result = handler(self, parcel, params)
                except Exception as e:
                    self.get_logger().error(f'Error in action "{action_type}": {e}')
                    result = GRAVEYARD_SIGNAL

            if result is GRAVEYARD_SIGNAL:
                self.send_parcel(parcel, graveyard)
                break  # Stop further processing

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
