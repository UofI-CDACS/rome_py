import json
from ...post_actions.registry import get_action
from ..base import InstructionSet
from ..types import InstructionResult, InstructionSignal
from ..registry import register_instruction_set


def get_parcel_data_value(parcel, key, default=None):
    for kv in parcel.data:
        if kv.key == key:
            return kv.value
    return default

def set_parcel_data_value(parcel, key, value):
    for kv in parcel.data:
        if kv.key == key:
            kv.value = value
            return
    # if not found, append new
    from post_interfaces.msg import KeyValue
    parcel.data.append(KeyValue(key=key, value=value))

@register_instruction_set("loop_dynamic")
class LoopDynamicInstructionSet(InstructionSet):
    graveyard = "default_graveyard"

    async def run(self, station, parcel) -> InstructionResult:
        # Bind actions
        dec_ttl = get_action("decrement_data_key")(station, parcel)
        check_ttl = get_action("check_ttl")(station, parcel)
        forward = get_action("forward")(station, parcel)

        await dec_ttl(key="ttl")

        if not await check_ttl(key="ttl"):
            return InstructionResult(signal=InstructionSignal.GRAVEYARD)

        # Read route JSON string from parcel data
        route_json = get_parcel_data_value(parcel, "route")
        if not route_json:
            return InstructionResult(signal=InstructionSignal.ERROR, notes="Missing 'route' in parcel data")

        try:
            route = json.loads(route_json)
            if not isinstance(route, list) or not route:
                raise ValueError()
        except Exception:
            return InstructionResult(signal=InstructionSignal.ERROR, notes="Invalid 'route' format in parcel data")

        # Get current index, default to 0
        index_str = get_parcel_data_value(parcel, "route_index", "0")
        try:
            index = int(index_str)
        except ValueError:
            index = 0

        # Current station name
        current_station_name = station.get_name().split("/")[-1]

        # Verify current station matches route[index] to catch errors
        if route[index] != current_station_name:
            return InstructionResult(signal=InstructionSignal.ERROR, notes="Current station does not match route index")

        # Calculate next index, wrap around
        next_index = (index + 1) % len(route)
        next_destination = route[next_index]

        # Update parcel data with next index
        set_parcel_data_value(parcel, "route_index", str(next_index))

        # Forward parcel
        await forward(destination=next_destination)

        return InstructionResult(signal=InstructionSignal.CONTINUE, next_destination=next_destination)
