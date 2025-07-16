from dataclasses import dataclass, field
from typing import Optional, Dict, Any
from .signals import InstructionSignal


@dataclass
class InstructionResult:
    signal: InstructionSignal
    notes: Optional[Dict[str, Any]] = field(default_factory=dict)
    next_destination: Optional[str] = None
