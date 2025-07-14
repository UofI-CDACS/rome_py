import inspect
from typing import Any
from rclpy.node import Node  # just for type hinting if you want

class StopAndForwardToGraveyard:
    pass

GRAVEYARD_SIGNAL = StopAndForwardToGraveyard()

ACTION_HANDLERS = {}

def action(name):
    def decorator(func):
        ACTION_HANDLERS[name] = func
        return func
    return decorator


async def _resolve_param(station: Node, parcel: Any, param: Any) -> Any:
    # Not an action, return as-is
    if not isinstance(param, dict) or 'run_action' not in param:
        return param

    action_name = param.get('run_action')
    handler = ACTION_HANDLERS.get(action_name)
    if not handler:
        station.get_logger().error(f"_resolve_param: Action '{action_name}' not found")
        return GRAVEYARD_SIGNAL

    nested_params = param.get('params', {})

    try:
        if inspect.iscoroutinefunction(handler):
            result = await handler(station, parcel, nested_params)
        else:
            result = handler(station, parcel, nested_params)
    except Exception as e:
        station.get_logger().error(f"_resolve_param: Error running action '{action_name}': {e}")
        return GRAVEYARD_SIGNAL

    return result

# Import your existing actions AFTER defining _resolve_param and ACTION_HANDLERS
from .log import *
from .wait import *
from .forward import *
from .check_ttl import *
from .conditional import *
from .decrement_data_key import *
from .kill import *
from .file_log import *
