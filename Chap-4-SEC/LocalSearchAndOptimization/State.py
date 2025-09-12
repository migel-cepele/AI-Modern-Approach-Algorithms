class State:
    def __init__(self, name, value = 0, parent = None):
        self.value = value
        self.parent = parent
        self.name = name