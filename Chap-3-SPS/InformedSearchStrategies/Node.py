class Node:
    def __init__(self, state, parent=None, pathCost=0, depth=0, h=0, f=0):
        self.state = state
        self.parent = parent
        self.pathCost = pathCost
        self.depth = depth
        self.h = h
        self.f = f
    
    # returns the path from root to the current node
    def path(self):
        node, path = self, []
        while node:
            path.append(node.state)
            node = node.parent
        return path[::-1]