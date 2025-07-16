from enum import Enum

class InstructionSignal(Enum):
    """Signals returned by instruction sets to guide station behavior."""
    CONTINUE = "continue"
    GRAVEYARD = "graveyard"
    ERROR = "error"
    RETRY = "retry"
    WAIT = "wait"

    def describe(self) -> str:
        descriptions = {
            self.CONTINUE: "Continue processing normally.",
            self.GRAVEYARD: "Send the parcel to the graveyard.",
            self.ERROR: "An error occurred during execution.",
            self.RETRY: "Retry the instruction set later.",
            self.WAIT: "Wait and resume later; station may decide how to reschedule.",
        }
        return descriptions.get(self, "Unknown signal.")

