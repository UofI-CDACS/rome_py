import asyncio
import rclpy
from rclpy.node import Node
from post_interfaces.msg import Parcel, StationKill
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ..post_actions.registry import get_action
import datetime as dt

import time
import psutil
import sys

import pymongo
database = pymongo.MongoClient("mongodb://root:example@172.23.254.20:27017/")

class Station(Node):
    def __init__(self, name=None, loss_mode='lossy', depth=10):
        super().__init__(f'{name}' or 'station_base')
        self.this_station = self.get_fully_qualified_name()
        self._pub_cache = {}
        #self._pub_lock = threading.Lock()
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
        self.get_logger().info(f'Station "{self.this_station}" started with {loss_mode} mode, listening for parcels.')
        # Publish kill signal AFTER subscriptions are set up
        kill_signal_pub = self.create_publisher(StationKill, f'{self.this_station}/kill', self.qos_profile)
        kill_signal = StationKill()
        kill_signal.kill_msg = f'All other stations with this name ({self.this_station}) must die!'
        kill_signal.timestamp = int(dt.datetime.now().timestamp())
        kill_signal_pub.publish(kill_signal)
        # Wait for 5 seconds to allow other stations to process kill signal
        time.sleep(5)
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
        
    def get_publisher(self, topic_name: str, qos_profile: QoSProfile):
        # Include QoS in the cache key
        qos_key = f"{topic_name}_{qos_profile.reliability}_{qos_profile.durability}_{qos_profile.depth}"

        #with self._pub_lock:
        if qos_key not in self._pub_cache:
            self._pub_cache[qos_key] = self.create_publisher(Parcel, topic_name, qos_profile)
        return self._pub_cache[qos_key]

    async def log_parcel(self, parcel):
        #Identifiers
        owner_id = getattr(parcel, 'owner_id', 'unknown')
        parcel_id = getattr(parcel, 'parcel_id', '<unknown>')

        prev_location = getattr(parcel, 'prev_location', '<unknown>')
        curr_location = self.get_fully_qualified_name()
        next_location = getattr(parcel, 'next_location', '<unknown>')

        #instruction_set  = getattr(parcel, 'instruction_set', '<unknown>')
        #data = getattr(parcel, 'data', {})

        #Analytics
        cpu_percent = psutil.cpu_percent(interval=None)
        cpu_temp = None
        try:
            temps = psutil.sensors_temperatures()
            if 'coretemp' in temps:
                cpu_temp = temps['coretemp'][0].current
            elif 'cpu_thermal' in temps:
                cpu_temp = temps['cpu_thermal'][0].current
        except:
            cpu_temp = "N/A"

        memory = psutil.virtual_memory()
        ram_percent = memory.percent
 
        # Network usage
        net_io = psutil.net_io_counters()
        bytes_sent_mb = net_io.bytes_sent / (1024 * 1024)
        bytes_recv_mb = net_io.bytes_recv / (1024 * 1024)
        parcel_size_mb = sys.getsizeof(parcel) / (1024 * 1024)
        
        timestamp_sent = getattr(parcel, 'timestamp_sent', None) 
        timestamp_recieved = getattr(parcel, 'timestamp_recieved', None) 

        database_data = {
            'timestamp_sent': timestamp_sent,
            'timestamp_recieved': timestamp_recieved,
            
            'parcel_id': parcel_id,
            'owner_id': owner_id,
            
            'prev_location': prev_location,
            'curr_location': curr_location,
            'next_location': next_location,

            'cpu_percent': cpu_percent,
            'cpu_temp': cpu_temp,
            'ram_percent': ram_percent,
            'bytes_sent_mb': bytes_sent_mb,
            'bytes_recv_mb': bytes_recv_mb,
            'parcel_size_mb': parcel_size_mb,
        }
 
        collection = database['logs']['logs']
        collection.insert_one(database_data)


    async def send_parcel(self, parcel, next_location: str, include_timestamp_rec=True):
        parcel.timestamp_sent = time.time_ns()
 
        # Create a new message of the same type
        published_parcel = type(parcel)()

        # Copy all fields from the original parcel
        for field_name in parcel.get_fields_and_field_types():
            setattr(published_parcel, field_name, getattr(parcel, field_name))

        # Update fields for publishing
        published_parcel.prev_location = self.get_fully_qualified_name()
        published_parcel.next_location = next_location

        topic = f'{next_location}/parcels'
        publisher = self.get_publisher(topic, self.qos_profile)
        publisher.publish(published_parcel)

        if not include_timestamp_rec:
            parcel.timestamp_recieved = None

        await self.log_parcel(parcel=parcel)
 
    def _on_parcel_received(self, parcel):
        parcel.timestamp_recieved = time.time_ns()

        self.get_logger().info(f"Parcel received callback triggered for parcel {parcel.parcel_id}")
        try:
            loop = asyncio.get_running_loop()
            loop.create_task(self.parcel_callback(parcel))
        except RuntimeError:
            # No event loop running, create one
            asyncio.ensure_future(self.parcel_callback(parcel))
   
    def _on_kill_signal(self, kill_signal):
        time_diff = int(dt.datetime.now().timestamp()) - kill_signal.timestamp
        self.get_logger().info(f"Received kill signal: {kill_signal.kill_msg} at {kill_signal.timestamp}")
        self.get_logger().info(f"Time since kill signal: {time_diff} seconds")
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

