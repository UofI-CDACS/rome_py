from abc import ABC, abstractmethod

class InstructionSet(ABC):
    def __init__(self):
        self.graveyard = 'default_graveyard'
        pass  # Optional setup

    @abstractmethod
    async def run(self, station, parcel):
        """Must be implemented by subclasses. Called with station and parcel."""
        raise NotImplementedError

