from post_station.actions import ACTION_HANDLERS, GRAVEYARD_SIGNAL
from post_station.instruction_sets.base import InstructionSet
from post_station.instruction_sets.registry import register_instruction_set

from post_station.instruction_sets.signals import InstructionSignal

@register_instruction_set("loop")
class LoopInstructionSet(InstructionSet):
    graveyard = "default_graveyard"

    async def run(self, station, parcel):
        name = station.get_name().split("/")[-1]
        await ACTION_HANDLERS["decrement_data_key"](station, parcel, {"key": "ttl"})
        await ACTION_HANDLERS["file_log_parcel"](station, parcel, {"log_path": "~/test_ws"})
       
        condition = await ACTION_HANDLERS["check_ttl"](station, parcel, {"key": "ttl"})

        if not condition:
            return InstructionSignal.GRAVEYARD
        if name == "rospi_1":
            ACTION_HANDLERS["forward"](station, parcel, {"destination": "rospi_2"})
        elif name == "rospi_2":
            ACTION_HANDLERS["forward"](station, parcel, {"destination": "rospi_3"})
        elif name == "rospi_3":
            ACTION_HANDLERS["forward"](station, parcel, {"destination": "rospi_4"})
        elif name == "rospi_4":
            ACTION_HANDLERS["forward"](station, parcel, {"destination": "rospi_1"})
        else:
            return InstructionSignal.ERROR

        return InstructionSignal.CONTINUE
