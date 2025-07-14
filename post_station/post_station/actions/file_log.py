import time
from post_station.actions import action
from rclpy.node import Node

@action('file_log_parcel')
def log_parcel(station, parcel, params):
    if not isinstance(station, Node):
        raise TypeError("Expected an rclpy Node instance")

    log_time = time.time_ns()
    qual_name = station.get_fully_qualified_name()
    parcel_id = getattr(parcel, 'parcel_id', '<unknown>')
    owner_id = getattr(parcel, 'owner_id', '<unknown>')
    prev_location = getattr(parcel, 'prev_location', '<unknown>')
    next_location = getattr(parcel, 'next_location', '<unknown>')
    instruction_set  = getattr(parcel, 'instruction_set', '<unknown>')
    data = getattr(parcel, 'data', '<unknown>')
    msg = (
        f"{log_time}, {qual_name}, {parcel_id}, {owner_id}, {prev_location}, {next_location}, {instruction_set}, {data}"
    )

    station.get_logger().info(msg)
