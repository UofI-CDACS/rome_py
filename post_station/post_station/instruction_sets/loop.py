from post_station.actions import ACTION_HANDLERS, GRAVEYARD_SIGNAL
from post_station.actions import _resolve_param
from post_station.instruction_sets.base import InstructionSet
from post_station.instruction_sets import instruction_set

@instruction_set("loop")
class LoopInstructionSet(InstructionSet):
    graveyard = "default_graveyard"

    async def run(self, station, parcel):
        name = station.get_name().split("/")[-1]
        if name == "rospi_1":
            await ACTION_HANDLERS["decrement_data_key"](station, parcel, {"key": "ttl"})
            condition = await ACTION_HANDLERS["check_ttl"](station, parcel, {"key": "ttl"})
            if condition:
                ACTION_HANDLERS["forward"](station, parcel, {"destination": "rospi_2"})
            else:
                res = GRAVEYARD_SIGNAL
            await ACTION_HANDLERS["file_log_parcel"](station, parcel, {"log_path": "~/test_ws"})
            return res

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

