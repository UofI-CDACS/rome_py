from ..registry import register_action
from post_station. import GRAVEYARD_SIGNAL

@register_action('kill')
def kill_action(station, parcel):
    return GRAVEYARD_SIGNAL
