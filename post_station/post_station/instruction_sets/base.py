from abc import ABC, abstractmethod

class InstructionSet(ABC):
    def __init__(self):
        self.graveyard = 'default_graveyard'

    @abstractmethod
    async def run(self, station, parcel):
        """Run the instruction set on given station and parcel."""
        raise NotImplementedError

