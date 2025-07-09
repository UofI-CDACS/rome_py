from post_station.actions import action

@action('log_parcel')
def log_parcel(node, parcel, params):
    msg = (
        f"[log_parcel] Parcel ID: {parcel.parcel_id}, "
        f"Owner: {parcel.owner_id}, "
        f"Location: {parcel.current_location}, "
        f"Arrived at: {parcel.next_destination}"
    )
    node.get_logger().info(msg)

