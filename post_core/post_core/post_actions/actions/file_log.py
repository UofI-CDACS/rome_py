import os
import socket
import time
import sys
from ..registry import register_action
from rclpy.node import Node
import psutil
import pymongo
import datetime
database = pymongo.MongoClient("mongodb://root:example@172.23.254.20:27017/")



@register_action('file_log_parcel')
async def file_log_parcel(station: Node, parcel, log_path: str, is_sender_log: bool):
    if not isinstance(station, Node):
        raise TypeError("Expected an rclpy Node instance")
    
    if not log_path:
        raise ValueError('file_log_parcel action requires a "log_path" parameter')
    
    log_folder = os.path.expanduser(log_path)
    os.makedirs(log_folder, exist_ok=True)

    owner_id = getattr(parcel, 'owner_id', 'unknown')
    hostname = socket.gethostname()
    #filename = f'log-{owner_id}-{hostname}.txt'
    #filepath = os.path.join(log_folder, filename)

    qual_name = station.get_fully_qualified_name()
    parcel_id = getattr(parcel, 'parcel_id', '<unknown>')
    prev_location = getattr(parcel, 'prev_location', '<unknown>')
    next_location = getattr(parcel, 'next_location', '<unknown>')
    instruction_set  = getattr(parcel, 'instruction_set', '<unknown>')
    data = getattr(parcel, 'data', '<unknown>')
    collection = database['logs']['logs']
    data_str = ';'.join(f"{kv.key}={kv.value}" for kv in data)

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
    log_time = time.time_ns()
    database_data = {
        'timestamp': log_time,
        'station_name': qual_name,
        'parcel_id': parcel_id,
        'owner_id': owner_id,
        'prev_location': prev_location,
        'next_location': next_location,
        'instruction_set': instruction_set,
        'cpu_percent': cpu_percent,
        'cpu_temp': cpu_temp,
        'ram_percent': ram_percent,
        'bytes_sent_mb': bytes_sent_mb,
        'bytes_recv_mb': bytes_recv_mb,
        'parcel_size_mb': parcel_size_mb,
        'sender': is_sender_log
    }
    for kv in data:
        if kv.key and kv.value:
            database_data[kv.key] = kv.value 
    collection.insert_one(database_data)

    # line = (
    #     f"{log_time},{qual_name},{parcel_id},{owner_id},{prev_location},"
    #     f"{next_location},{instruction_set},{cpu_percent},"
    #     f"{cpu_temp},{ram_percent},{bytes_sent_mb},{bytes_recv_mb},{parcel_size_mb},{data_str}\n"
    # )
    # try:
    #     with open(filepath, 'a', encoding='utf-8') as f:
    #         f.write(line)
    # except Exception as e:
    #     raise ValueError(f'Failed to write log file "{filepath}": {e}')
