from typing import Dict, Callable, Any
from .base import ActionHandler

_action_registry: Dict[str, Callable[..., ActionHandler]] = {}

def register_action(name: str):
    def decorator(func: Callable[..., ActionHandler]):
        if name in _action_registry:
            raise ValueError(f"Action '{name}' already registered")
        _action_registry[name] = func
        return func
    return decorator

def get_action(name: str) -> Callable[..., ActionHandler]:
    return _action_registry.get(name)
