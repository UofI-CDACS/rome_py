from abc import ABC, abstractmethod
from .result import InstructionResult

class InstructionSet(ABC):
    def __init__(self):
        self.graveyard = 'default_graveyard'

    @abstractmethod
    async def run(self, station, parcel) -> InstructionResult:
        """Run the instruction set on given station and parcel."""
        raise NotImplementedError

