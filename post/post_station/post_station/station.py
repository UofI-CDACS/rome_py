from ament_index_python.packages import get_package_share_directory
import os
import yaml
import rclpy
from rclpy.node import Node
from post_interfaces.msg import Parcel

class Station(Node):
    def __init__(self):
        super().__init__('station_default')
        self.this_station = self.get_fully_qualified_name()
        self._instruction_sets_cache = {}
        self.subscription = self.create_subscription(
            Parcel,
            f'{self.this_station}/parcels',
            self.parcel_callback,
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
        if topic_name not in self.publishers:
            self._pub_cache[topic_name] = self.create_publisher(Parcel, topic_name, 10)
            self.get_logger().info(f'Created publisher for topic: {topic_name}')
        return self._pub_cache[topic_name]

    def parcel_callback(self, parcel: Parcel):
        if parcel.next_destination != self.this_station:
            self.get_logger().warn(
                f'Parcel {parcel.parcel_id} not intended for this station ({self.this_station}). Ignoring.'
            )
            return

        self.get_logger().info(f'Received parcel {parcel.parcel_id} at {self.this_station}')
        
        instruction_set = self.get_instruction_set(parcel.instruction_set_id)
        actions = instruction_set.get(self.this_station, [])
       
        if not actions:
            self.get_logger().info(f'No actions defined for this parcel at this station.')
        else:
            self.get_logger().info(f'Executing {len(actions)} actions for parcel {parcel.parcel_id}')
            for action in actions:
                self.get_logger().info(f'Would execute action: {action}')
                # TODO: call action handler here

        # TEMP: No action effects yet — avoid infinite loop
        self.get_logger().info(f'Parcel {parcel.parcel_id} processing complete (no forward).')


def main(args=None):
    rclpy.init(args=args)
    node = Station()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

