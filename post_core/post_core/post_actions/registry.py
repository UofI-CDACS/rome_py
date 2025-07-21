from typing import Dict, Callable, Any
from .base import ActionHandler
from .base import action

_registry: Dict[str, Callable[..., ActionHandler]] = {}

def register_action(name: str):
    def decorator(func: Callable[..., ActionHandler]):
        wrapped_func = action(func)
        if name in _registry:
            raise ValueError(f"Action '{name}' already registered")
        _registry[name] = wrapped_func
        return wrapped_func
    return decorator

def get_action(name: str) -> Callable[..., ActionHandler]:
    cls = _registry.get(name)
    if cls is None:
        raise ValueError(f"Action type '{name}' is not registered.")
    return cls
