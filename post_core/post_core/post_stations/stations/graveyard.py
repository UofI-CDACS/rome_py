from ..base import Station
from ..registry import register_station
from ...post_instruction_sets.registry import get_instruction_set
from post_interfaces.msg import Parcel
from ...post_instruction_sets.types import InstructionResult, InstructionSignal

@register_station("graveyard")
class GraveyardStation(Station):

    async def parcel_callback(self, parcel: Parcel):
        self.get_logger().info(f"Received parcel {parcel.parcel_id} for processing.")

        InstructionSetClass = get_instruction_set('graveyard')
        if InstructionSetClass is None:
            self.get_logger().warn(f"No instruction set found for graveyard")
            return

        self.get_logger().info(f"Running instruction set '{parcel.instruction_set}' for parcel {parcel.parcel_id}")
        instruction_set = InstructionSetClass()
        await instruction_set.run(self, parcel)
        self.get_logger().info(f"Parcel {parcel.parcel_id} processed successfully.")

