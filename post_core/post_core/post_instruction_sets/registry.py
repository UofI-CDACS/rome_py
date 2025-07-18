from typing import Type, Dict
from .base import InstructionSet

_registry: Dict[str, Type[InstructionSet]] = {}

def register_instruction_set(name: str):
    def decorator(cls: Type[InstructionSet]):
        if name in _registry:
            raise ValueError(f"Instruction set '{name}' already registered")
        _registry[name] = cls
        return cls
    return decorator

def get_instruction_set(name: str) -> Type[InstructionSet]:
    return _registry.get(name)
