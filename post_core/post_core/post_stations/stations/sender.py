import json
import uuid
from rclpy.parameter import Parameter, ParameterType
from post_interfaces.msg import KeyValue, Parcel
from post_core.post_stations.base import Station
from ..registry import register_station
from post_core.post_actions import get_action
from rcl_interfaces.msg import SetParametersResult
import asyncio
#import threading

@register_station("sender")
class SenderStation(Station):
    def __init__(self, name, loss_mode='lossy', depth=10):
        # Pass loss_mode and depth to parent constructor
        super().__init__(name, loss_mode=loss_mode, depth=depth)

        # Declare parameters with defaults
        self.declare_parameter('destinations', ['rospi_1', 'rospi_2', 'rospi_3', 'rospi_4'])
        self.declare_parameter('count', 20)
        self.declare_parameter('mode', 'round_robin')  # could be 'round_robin', 'random', 'once'
        self.declare_parameter('interval_sec', 1.0)
        self.declare_parameter('owner_id', 'sender_station')
        self.declare_parameter('instruction_set', 'default')
        self.declare_parameter('data', ['ttl:10'])
        
        # Read initial values
        self.destinations = self.get_parameter('destinations').get_parameter_value().string_array_value
        self.count = self.get_parameter('count').get_parameter_value().integer_value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value.lower()
        
        # Index to track round robin position
        self._rr_index = 0
        
        # Setup timer to send parcels at interval
        self.timer = self.create_timer(
            self.get_parameter('interval_sec').get_parameter_value().double_value,
            self.publish_parcel
        )
        
        # Subscribe to param updates
        self.add_on_set_parameters_callback(self._on_params_changed)
        
        # Tracking how many parcels sent
        self._sent_count = 0
        #self._state_lock = threading.Lock()

    def _on_params_changed(self, params):
        #with self._state_lock:
        for param in params:
            if param.name == 'destinations':
                self.destinations = param.value
                self.get_logger().info(f"Updated destinations: {self.destinations}")
            elif param.name == 'count':
                self.count = param.value
                self.get_logger().info(f"Updated count: {self.count}")
            elif param.name == 'mode':
                self.mode = param.value.lower()
                self.get_logger().info(f"Updated mode: {self.mode}")
            
            # Reset counters on param change if needed
            self._sent_count = 0
            self._rr_index = 0
        return SetParametersResult(successful=True)
        
    def publish_parcel(self):
        #with self._state_lock:
        if self._sent_count >= self.count:
            self.get_logger().info(f"Sent all {self.count} parcels. Stopping.")
            self.timer.cancel()
            raise SystemExit
        
        # Move all destination logic inside the lock
        if self.mode == 'round_robin':
            destination = self.destinations[self._rr_index % len(self.destinations)]
            self._rr_index += 1
        elif self.mode == 'random':
            import random
            destination = random.choice(self.destinations)
        elif self.mode == 'once':
            if self._sent_count < len(self.destinations):
                destination = self.destinations[self._sent_count]
            else:
                self.get_logger().info("Once mode: all destinations used, stopping.")
                self.timer.cancel()
                return
        else:
            self.get_logger().warn(f"Unknown mode '{self.mode}', defaulting to round_robin.")
            destination = self.destinations[self._rr_index % len(self.destinations)]
            self._rr_index += 1
        
        parcel = Parcel()
        parcel.parcel_id = str(uuid.uuid4())
        parcel.owner_id = self.get_parameter('owner_id').get_parameter_value().string_value
        parcel.instruction_set = self.get_parameter('instruction_set').get_parameter_value().string_value
        
        namespace = self.get_namespace().strip('/')
        destination = destination.strip('/')
        full_destination = f"/{namespace}/{destination}" if namespace else f"/{destination}"
        
        raw_data = self.get_parameter('data').get_parameter_value().string_array_value
        for entry in raw_data:
            try:
                key, value = entry.split(':', 1)
                kv = KeyValue(key=key.strip(), value=value.strip())
                parcel.data.append(kv)
            except ValueError:
                self.get_logger().warn(f"Invalid data entry: '{entry}'")
 
        try:
            loop = asyncio.get_running_loop()
            # Create task for async sending
            loop.create_task(self._send_parcel_async(parcel, full_destination))
        except RuntimeError:
            # Fallback if no loop (shouldn't happen in ROS2)
            self.get_logger().error(f"Failed to send parcel: No event loop found for async sending")



    async def _send_parcel_async(self, parcel, destination):
        try:
            await self.send_parcel(parcel, destination)
            self.get_logger().info(f"Sent parcel {self._sent_count}/{self.count} to {destination}")
            self._sent_count += 1
        except Exception as e:
            self.get_logger().error(f"Failed to send parcel: {e}")
