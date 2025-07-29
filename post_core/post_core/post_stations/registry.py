from typing import Callable, Dict, Type
from .base import Station

_registry: Dict[str, Type[Station]] = {}

def register_station(name: str):
    def decorator(cls: Type[Station]):
        _registry[name] = cls
        return cls
    return decorator

def get_station_class(name: str) -> Type[Station]:
    cls = _registry.get(name)
    if cls is None:
        raise ValueError(f"Station type '{name}' is not registered.")
    return cls    

def all_stations() -> Dict[str, Type[Station]]:
    return _station_registry.copy()
