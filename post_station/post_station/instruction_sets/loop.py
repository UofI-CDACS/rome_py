from post_station.actions import ACTION_HANDLERS, GRAVEYARD_SIGNAL
from post_station.actions import _resolve_param
from post_station.instruction_sets.base import InstructionSet
from post_station.instruction_sets.registry import register_instruction_set

@register_instruction_set("loop")
class LoopInstructionSet(InstructionSet):
    graveyard = "default_graveyard"

    async def run(self, station, parcel):
        name = station.get_name().split("/")[-1]
        if name == "rospi_1":
            await ACTION_HANDLERS["decrement_data_key"](station, parcel, {"key": "ttl"})
            condition = await ACTION_HANDLERS["check_ttl"](station, parcel, {"key": "ttl"})
            send_to_graveyard = False
            if condition:
                ACTION_HANDLERS["forward"](station, parcel, {"destination": "rospi_2"})
            else:
                send_to_graveyard = True
            await ACTION_HANDLERS["file_log_parcel"](station, parcel, {"log_path": "~/test_ws"})
            if send_to_graveyard:
                return GRAVEYARD_SIGNAL

        elif name == "rospi_2":
            await ACTION_HANDLERS["decrement_data_key"](station, parcel, {"key": "ttl"})
            await ACTION_HANDLERS["file_log_parcel"](station, parcel, {"log_path": "~/test_ws"})
            condition = await ACTION_HANDLERS["check_ttl"](station, parcel, {"key": "ttl"})
            if condition:
                ACTION_HANDLERS["forward"](station, parcel, {"destination": "rospi_3"})
            else:
                return GRAVEYARD_SIGNAL

        elif name == "rospi_3":
            await ACTION_HANDLERS["decrement_data_key"](station, parcel, {"key": "ttl"})
            await ACTION_HANDLERS["file_log_parcel"](station, parcel, {"log_path": "~/test_ws"})
            condition = await ACTION_HANDLERS["check_ttl"](station, parcel, {"key": "ttl"})
            if condition:
                ACTION_HANDLERS["forward"](station, parcel, {"destination": "rospi_4"})
            else:
                return GRAVEYARD_SIGNAL

        elif name == "rospi_4":
            await ACTION_HANDLERS["decrement_data_key"](station, parcel, {"key": "ttl"})
            await ACTION_HANDLERS["file_log_parcel"](station, parcel, {"log_path": "~/test_ws"})
            condition = await ACTION_HANDLERS["check_ttl"](station, parcel, {"key": "ttl"})
            if condition:
                ACTION_HANDLERS["forward"](station, parcel, {"destination": "rospi_1"})
            else:
                return GRAVEYARD_SIGNAL

        return True

