from enum import Enum
from dataclasses import dataclass
from typing import Optional

class StationSignal(Enum):
    SUCCESS = "success"
    ERROR = "error"
    TIMEOUT = "timeout"
    SHUTDOWN = "shutdown"

@dataclass
class StationResult:
    signal: StationSignal
    notes: Optional[str] = None
