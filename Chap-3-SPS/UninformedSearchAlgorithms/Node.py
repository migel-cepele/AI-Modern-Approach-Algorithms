class Node:
    def __init__(self, state, parent=None, pathCost=0, depth=0):
        self.state = state
        self.parent = parent
        self.pathCost = pathCost
        self.depth = depth
    
    # returns the path from root to the current node
    def path(self):
        node, path = self, []
        while node:
            path.append(node.state)
            node = node.parent
        return path[::-1]