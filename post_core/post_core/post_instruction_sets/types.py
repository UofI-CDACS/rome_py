from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional

class InstructionSignal(Enum):
    CONTINUE = auto()
    GRAVEYARD = auto()
    RETRY = auto()
    ERROR = auto()

@dataclass
class InstructionResult:
    signal: InstructionSignal
    next_destination: Optional[str] = None
    notes: Optional[str] = None
