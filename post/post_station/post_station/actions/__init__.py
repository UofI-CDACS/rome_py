# post_station/actions/__init__.py

ACTION_HANDLERS = {}

def action(name):
    def decorator(func):
        ACTION_HANDLERS[name] = func
        return func
    return decorator

from .log import *
from .wait import *
