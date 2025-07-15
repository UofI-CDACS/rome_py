from post_station.actions import action, GRAVEYARD_SIGNAL

@action('kill')
def kill_action(station, parcel, params):
    #station.get_logger().warn(f"Parcel {parcel.parcel_id} killed.")
    return GRAVEYARD_SIGNAL
