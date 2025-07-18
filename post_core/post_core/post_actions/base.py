from typing import Callable, Awaitable, Any
from rclpy.node import Node

# Define the type of an action handler: async or sync callable
ActionHandler = Callable[..., Awaitable[Any]]

def action(func: Callable[..., Any]) -> Callable[..., ActionHandler]:
    """
    Decorator to wrap a function into an action factory that returns
    an async runner with station and parcel bound.
    """
    def factory(station: Node, parcel: Any) -> ActionHandler:
        async def runner(*args, **kwargs):
            return await func(station, parcel, *args, **kwargs)
        return runner
    return factory
