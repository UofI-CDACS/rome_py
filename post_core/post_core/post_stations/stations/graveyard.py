from ..base import Station
from ..registry import register_station
from ...post_instruction_sets.registry import get_instruction_set
from post_interfaces.msg import Parcel
from ...post_instruction_sets.types import InstructionResult, InstructionSignal

@register_station("graveyard")
class GraveyardStation(Station):

    async def parcel_callback(self, parcel: Parcel):
        self.get_logger().info(f"Received parcel {parcel.parcel_id} for processing.")
        parcel.next_location = None
        parcel.timestamp_sent = None
        await self.log_parcel(
            parcel = parcel,
        )
        self.get_logger().info(f"Parcel {parcel.parcel_id} processed successfully.")

