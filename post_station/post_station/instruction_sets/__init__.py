INSTRUCTION_SETS = {}

def instruction_set(name):
    def decorator(cls):
        INSTRUCTION_SETS[name] = cls
        return cls
    return decorator

from .loop import *

