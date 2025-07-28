import asyncio
import rclpy
from rclpy.node import Node
from post_interfaces.msg import Parcel
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import threading

class Station(Node):
    def __init__(self, name=None):
        super().__init__(f'{name}' or 'station_base')
        self.this_station = self.get_fully_qualified_name()
        self._pub_cache = {}
        self._lock = threading.Lock()
        self.subscription = self.create_subscription(
            Parcel,
            f'{self.this_station}/parcels',
            self._on_parcel_received,
            10
        )
        self.get_logger().info(f'Station "{self.this_station}" started, listening for parcels.')

    def get_publisher(self, topic_name: str, qos_profile: QoSProfile):
        with self._pub_lock:
            if topic_name not in self._pub_cache:
                self._pub_cache[topic_name] = self.create_publisher(Parcel, topic_name, qos_profile)
            return self._pub_cache[topic_name]

    def send_parcel(self, parcel, next_location: str, lossmode: str = 'lossy'):
        if lossmode == 'lossless':
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1000,
                history=HistoryPolicy.KEEP_ALL 
            )
        else:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
                history=HistoryPolicy.KEEP_LAST
            )
        topic = f'{next_location}/parcels'
        publisher = self.get_publisher(topic, qos_profile)
        publisher.publish(parcel)

    def _on_parcel_received(self, parcel):
        self.get_logger().info(f"Parcel received callback triggered for parcel {parcel.parcel_id}")
        try:
            loop = asyncio.get_running_loop()
            loop.create_task(self.parcel_callback(parcel))
        except RuntimeError:
            # No event loop running, create one
            asyncio.ensure_future(self.parcel_callback(parcel))
    
    async def parcel_callback(self, parcel: Parcel):
        # To be overridden by subclasses
        self.get_logger().warn(f"Received parcel {parcel.parcel_id} but no handler implemented.")
