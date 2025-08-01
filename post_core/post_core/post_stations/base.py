import asyncio
import rclpy
from rclpy.node import Node
from post_interfaces.msg import Parcel, StationKill
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import threading
import datetime as dt

class Station(Node):
    def __init__(self, name=None, loss_mode='lossy', depth=10):
        super().__init__(f'{name}' or 'station_base')
        self.this_station = self.get_fully_qualified_name()
        self._pub_cache = {}
        self._pub_lock = threading.Lock()
        self.loss_mode = loss_mode
        self.depth = depth
        
        if loss_mode == 'lossless':
            self.qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=depth,
                history=HistoryPolicy.KEEP_ALL
            )
        else:
            self.qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=depth,
                history=HistoryPolicy.KEEP_LAST
            )
        
        # Create parcel subscription
        self.parcel_subscription = self.create_subscription(
            Parcel,
            f'{self.this_station}/parcels',
            self._on_parcel_received,
            self.qos_profile
        )

        # Create kill signal subscription with separate variable name
        self.kill_subscription = self.create_subscription(
            StationKill,
            f'{self.this_station}/kill',
            self._on_kill_signal,
            self.qos_profile
        )

        self.get_logger().info(f'Station "{self.this_station}" started with {loss_mode} mode, listening for parcels.')

        # Publish kill signal AFTER subscriptions are set up
        kill_signal_pub = self.create_publisher(StationKill, f'{self.this_station}/kill', self.qos_profile)
        kill_signal = StationKill()
        kill_signal.kill_msg = f'All other stations with this name ({self.this_station}) must die!'
        kill_signal.timestamp = int(dt.datetime.now().timestamp())
        kill_signal_pub.publish(kill_signal)
        
    def get_publisher(self, topic_name: str, qos_profile: QoSProfile):
        # Include QoS in the cache key
        qos_key = f"{topic_name}_{qos_profile.reliability}_{qos_profile.durability}_{qos_profile.depth}"
        
        with self._pub_lock:
            if qos_key not in self._pub_cache:
                self._pub_cache[qos_key] = self.create_publisher(Parcel, topic_name, qos_profile)
            return self._pub_cache[qos_key]

    def send_parcel(self, parcel, next_location: str):
        topic = f'{next_location}/parcels'
        publisher = self.get_publisher(topic, self.qos_profile)
        publisher.publish(parcel)

    def _on_parcel_received(self, parcel):
        self.get_logger().info(f"Parcel received callback triggered for parcel {parcel.parcel_id}")
        try:
            loop = asyncio.get_running_loop()
            loop.create_task(self.parcel_callback(parcel))
        except RuntimeError:
            # No event loop running, create one
            asyncio.ensure_future(self.parcel_callback(parcel))
   
    def _on_kill_signal(self, kill_signal):
        time_diff = int(dt.datetime.now().timestamp()) - kill_signal.timestamp
        if kill_signal.kill_msg == f'All other stations with this name ({self.this_station}) must die!':
            return  # Ignore our own kill signal
        if time_diff < 5:  # Only process recent kill signals (within 5 seconds)
            self.get_logger().info(f"Kill Message: {kill_signal.kill_msg}")
            raise SystemExit

    async def parcel_callback(self, parcel: Parcel):
        # To be overridden by subclasses
        self.get_logger().warn(f"Received parcel {parcel.parcel_id} but no handler implemented.")
    
    def __del__(self):
        # Clean up publishers to prevent memory leaks
        try:
            for pub in self._pub_cache.values():
                pub.destroy()
            self._pub_cache.clear()
        except:
            pass  # Ignore cleanup errors during shutdown
