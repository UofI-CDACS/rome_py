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
from post_station.instruction_sets import INSTRUCTION_SETS
from post_station.instruction_sets.signals import InstructionSignal
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
        self.get_logger().info(f'Station "{self.this_station}" started, listening ... for parcels.')
        #self.get_logger().info(f'Loaded instruction sets: {INSTRUCTION_SETS.keys()}')

    def get_instruction_set(self, set_id: str):
        instruction_set = INSTRUCTION_SETS.get(set_id)
        if not instruction_set:
            self.get_logger().error(f'Unknown instruction set: {set_id}')
            return None
        return instruction_set  # Create fresh instance 

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
        instruction_set = self.get_instruction_set(parcel.instruction_set)
        if instruction_set is None:
            self.get_logger().warn(f"No instruction set found for {parcel.instruction_set}")
            return

        graveyard = getattr(instruction_set, 'graveyard', 'default_graveyard') 
        
        try:
            result = await instruction_set.run(self, parcel)

            if result == InstructionSignal.GRAVEYARD:
                self.get_logger().warn(f"Parcel {parcel.parcel_id} killed. Sending to graveyard.")
                self.send_parcel(parcel, graveyard)
            elif result == InstructionSignal.RETRY:
                self.get_logger().warn(f"Parcel {parcel.parcel_id} retry requested. Retry not defined at this time.")
            elif result == InstructionSignal.ERROR:
                self.get_logger().error(f"Parcel {parcel.parcel_id} encountered an error signal.")
                self.send_parcel(parcel, graveyard)
            else:
                # CONTINUE or other signals - assume normal flow, do nothing or logging
                self.get_logger().info(f"Parcel {parcel.parcel_id} processed successfully.")
        except Exception as e:
            self.get_logger().error(f"Instruction set error: {e}")
            self.get_logger().warn(f"Parcel {parcel.parcel_id} killed due to exception.")
            self.send_parcel(parcel, graveyard)

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
