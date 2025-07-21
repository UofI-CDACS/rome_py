from ...post_actions.registry import get_action
from ..base import InstructionSet
from ..types import InstructionResult, InstructionSignal
from ..registry import register_instruction_set 
import asyncio

@register_instruction_set("default")
class DefaultInstructionSet(InstructionSet):
    async def run(self, station, parcel) -> InstructionResult:
        dec_ttl = get_action('decrement_data_key')(station, parcel)
        log_parcel = get_action('file_log_parcel')(station, parcel)
        check_ttl = get_action('check_ttl')(station, parcel)
        forward = get_action('forward')(station, parcel)

        await dec_ttl(key="ttl")
        await log_parcel(log_path=f"~/test_ws/default/{station.this_station}")

        if not await check_ttl(key="ttl"):
            return InstructionResult(signal=InstructionSignal.GRAVEYARD)

        await asyncio.sleep(0.01)
        destination = await forward(destination=station.get_name().split("/")[-1])
        print(f'destination: {destination}')
        return InstructionResult(next_destination = destination, signal=InstructionSignal.CONTINUE)

