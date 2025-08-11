from ...post_actions.registry import get_action
from ..base import InstructionSet
from ..types import InstructionResult, InstructionSignal
from ..registry import register_instruction_set

@register_instruction_set("loop")
class LoopInstructionSet(InstructionSet):
    graveyard = "default_graveyard"
    async def run(self, station, parcel) -> InstructionResult:
        # Bind actions to this station and parcel
        dec_ttl = get_action('decrement_data_key')(station, parcel)
        log_parcel = get_action('file_log_parcel')(station, parcel)
        check = get_action('check_ttl')(station, parcel)
        fwd = get_action('forward')(station, parcel)

        # Map current station to the next destination
        next_map = {
            "rospi_1": "rospi_2",
            "rospi_2": "rospi_3",
            "rospi_3": "rospi_4",
            "rospi_4": "rospi_1",
        }

        name = station.get_name().split("/")[-1]

        await dec_ttl(key="ttl")
        #log_parcel = get_action('file_log_parcel')(station, parcel)
        await log_parcel(log_path=f"~/Desktop/test_ws/loop/{station.this_station}", is_sender_log=False)

        if not await check(key="ttl"):
            return InstructionResult(signal=InstructionSignal.GRAVEYARD)

        destination = next_map.get(name)
        if destination is None:
            return InstructionResult(signal=InstructionSignal.ERROR, notes=f"Unknown station name: {name}")

        await fwd(destination=destination)

        return InstructionResult(signal=InstructionSignal.CONTINUE, next_destination=destination)
