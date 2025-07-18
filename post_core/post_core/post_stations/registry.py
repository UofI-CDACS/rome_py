from typing import Callable, Dict, Type
from .base import Station

_station_registry: Dict[str, Type[Station]] = {}

def register_station(name: str):
    def decorator(cls: Type[Station]):
        _station_registry[name] = cls
        return cls
    return decorator

def get_station_class(name: str) -> Type[Station]:
    return _station_registry.get(name)

def all_stations() -> Dict[str, Type[Station]]:
    return _station_registry.copy()
