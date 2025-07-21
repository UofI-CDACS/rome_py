from ...post_actions.registry import get_action
from ..base import InstructionSet
from ..types import InstructionResult, InstructionSignal
from ..registry import register_instruction_set 
import asyncio

@register_instruction_set("graveyard")
class GraveyardInstructionSet(InstructionSet):
    async def run(self, station, parcel) -> InstructionResult:
        log_parcel = get_action('file_log_parcel')(station, parcel)
        await log_parcel(log_path=f"~/test_ws/graveyard/{station.this_station}")
        return InstructionResult(signal=InstructionSignal.CONTINUE)
