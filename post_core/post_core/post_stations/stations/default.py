from ..base import Station
from ..registry import register_station
from ...post_instruction_sets.registry import get_instruction_set
from post_interfaces.msg import Parcel
from ...post_instruction_sets.types import InstructionResult, InstructionSignal

@register_station("default")
class DefaultStation(Station):

    async def parcel_callback(self, parcel: Parcel):
        self.get_logger().info(f"Received parcel {parcel.parcel_id} for processing.")
        if parcel.next_location != self.this_station:
            self.get_logger().warn(
                f'Parcel {parcel.parcel_id} not for this station ({self.this_station}). Ignoring.'
            )
            return

        InstructionSetClass = get_instruction_set(parcel.instruction_set)
        if InstructionSetClass is None:
            self.get_logger().warn(f"No instruction set found for {parcel.instruction_set}")
            return

        self.get_logger().info(f"Running instruction set '{parcel.instruction_set}' for parcel {parcel.parcel_id}")
        instruction_set = InstructionSetClass()
        graveyard = getattr(instruction_set, 'graveyard', 'default_graveyard')

        try:
            result = await instruction_set.run(self, parcel)

            if result.signal == InstructionSignal.GRAVEYARD:
                self.get_logger().warn(f"Parcel {parcel.parcel_id} killed. Sending to graveyard: {graveyard}.")
                self.send_parcel(parcel, graveyard)
            elif result.signal == InstructionSignal.RETRY:
                self.get_logger().warn(f"Parcel {parcel.parcel_id} retry requested. Retry not implemented.")
            elif result.signal == InstructionSignal.ERROR:
                self.get_logger().error(f"Parcel {parcel.parcel_id} error signal received. Notes: {result.notes}")
                self.send_parcel(parcel, graveyard)
            else:
                if result.next_destination:
                    self.get_logger().info(f"Parcel {parcel.parcel_id} forwarding to {result.next_destination}")
                    self.send_parcel(parcel, result.next_destination)
                else:
                    self.get_logger().info(f"Parcel {parcel.parcel_id} processed successfully.")

        except Exception as e:
            self.get_logger().error(f"Instruction set error: {e}")
            self.get_logger().warn(f"Parcel {parcel.parcel_id} killed due to exception.")
            self.send_parcel(parcel, graveyard)
