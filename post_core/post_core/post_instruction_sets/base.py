from abc import ABC, abstractmethod
from typing import Any
from .types import InstructionResult

class InstructionSet(ABC):
    graveyard: str = "default_graveyard"

    @abstractmethod
    async def run(self, station: Any, parcel: Any) -> InstructionResult:
        """
        Execute the instruction set.

        Parameters:
        - station: the ROS node station processing the parcel
        - parcel: the data/message parcel

        Returns:
        - InstructionResult indicating the outcome
        """
        pass
