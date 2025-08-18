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

        # Read route map JSON string from parcel data
        route_json = get_parcel_data_value(parcel, "route")
        if not route_json:
            # Fallback to default route map if not provided
            route_map = {
                "rospi_1": "rospi_2",
                "rospi_2": "rospi_3", 
                "rospi_3": "rospi_4",
                "rospi_4": "rospi_1"
            }
        else:
            try:
                route_map = json.loads(route_json)
                if not isinstance(route_map, dict) or not route_map:
                    raise ValueError("Route must be a non-empty object/map")
            except Exception as e:
                return InstructionResult(signal=InstructionSignal.ERROR, notes=f"Invalid 'route' format: {e}")

        # Current station name (strip namespace)
        current_station_name = station.get_name().split("/")[-1]

        # Get next destination from route map
        next_destination = route_map.get(current_station_name)
        if next_destination is None:
            return InstructionResult(signal=InstructionSignal.ERROR, 
                                   notes=f"Current station '{current_station_name}' not found in route map {route_map}")

        # Forward parcel
        await forward(destination=next_destination)

        return InstructionResult(signal=InstructionSignal.CONTINUE, next_destination=next_destination)
