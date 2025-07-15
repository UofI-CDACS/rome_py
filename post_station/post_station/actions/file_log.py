import os
import socket
import time
from post_station.actions import action, _resolve_param
from rclpy.node import Node

@action('file_log_parcel')
async def file_log_parcel(station, parcel, params):
    if not isinstance(station, Node):
        raise TypeError("Expected an rclpy Node instance")
   
    log_folder= await _resolve_param(station, parcel, params.get('log_path', ''))
    if not log_folder:
        raise ValueError('file_log_parcel action requires a "log_path" parameter')

    # Create folder if it doesn't exist
    log_folder = os.path.expanduser(log_folder)
    os.makedirs(log_folder, exist_ok=True)

    owner_id = getattr(parcel, 'owner_id', 'unknown')
    hostname = socket.gethostname()
    filename = f'log-{owner_id}-{hostname}.txt'
    filepath = os.path.join(log_folder, filename)

    log_time = time.time_ns()
    qual_name = station.get_fully_qualified_name()
    parcel_id = getattr(parcel, 'parcel_id', '<unknown>')
    prev_location = getattr(parcel, 'prev_location', '<unknown>')
    next_location = getattr(parcel, 'next_location', '<unknown>')
    instruction_set  = getattr(parcel, 'instruction_set', '<unknown>')
    data = getattr(parcel, 'data', '<unknown>')
    data_str = ';'.join(f"{kv.key}={kv.value}" for kv in data)

    # Compose CSV-style line without header
    line = (
        f"{log_time},{qual_name},{parcel_id},{owner_id},{prev_location},"
        f"{next_location},{instruction_set},{data_str}\n"
    )

    try:
        with open(filepath, 'a', encoding='utf-8') as f:
            f.write(line)
    except Exception as e:
        raise ValueError(f'Failed to write log file "{filepath}": {e}')
