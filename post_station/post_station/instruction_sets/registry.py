INSTRUCTION_SET_REGISTRY = {}

def register_instruction_set(name):
    def decorator(set):
        INSTRUCTION_SET_REGISTRY[name] = set()
        return set
    return decorator

