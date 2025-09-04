from Node import Node
import numpy as np

def evaluationFunction(frontier):
        return np.argmax([node.depth for node in frontier])



class IDS:
    def __init__(self, s_start, goal, actions, action_costs, states, evaluation=None):
        self.start = Node(s_start)
        self.goal = goal
        self.actions = actions
        self.action_costs = action_costs
        self.states = states
        self.evaluationFunction = evaluation if evaluation else self.evaluationFunction

    def iterate(self):
         depth = 0
         while True:
            print('Iterating for depth ', depth)
            path, pathCost = self.depthLimitedSearch(depth)
            depth += 1
            if path != None:
                return path, pathCost
            if depth > 9:
                print("Depth is ", depth)
                return None
              

    def depthLimitedSearch(self, l):
        print("IDS Search")
        frontier = [self.start] # initialize the frontier with the start node   

        while len(frontier) > 0:
            #frontier is a priority queue based on path cost
            max_index = self.evaluationFunction(frontier)
            node = frontier.pop(max_index)
            
            if self.goal == node.state:
                 print("Goal reached ", node.state)
                 return node.path(), node.pathCost
            
            if node.depth > l:
                 return None, float('inf')
            elif not self.isCycle(node):
                children = self.expand(node) # expand the node to get its children
                for child in children:                
                    frontier.append(child)
                    print("Node was added to frontier: ", child.state)

        return None, float('inf') # return None if the goal is not reachable
    

    # by default the evaluation function returns the index of the node with the lowest path cost
    def evaluationFunction(self, frontier):
        return np.argmin([node.pathCost for node in frontier])


    def expand(self, node):
        s = node.state
        print("Node to expand: ", s)
        
        actions_s = self.actions[s]
        print("Possible actions: ", self.actions[s])

        action_costs_s = self.action_costs[s]
        print("Action costs: ", self.action_costs[s])

        expanded_nodes = []

        for i in range(len(actions_s)):
            if actions_s[i] == -1:
                continue
            cost = node.pathCost + action_costs_s[i] # get the cost of the action
            new_node = Node(actions_s[i], node, cost, node.depth + 1)
            expanded_nodes.append(new_node) # add the new node
    
        print("Expanded nodes: ", expanded_nodes)
        return expanded_nodes
    
    def isCycle(self, node):
        s = node.state
        p = node.parent
        while p != None:
             if p.state == s: return True
             p = p.parent
        return False

# Example route finding problem in albania
states = np.array(["Tirana", "Durres", "Kavaje", "Vlore", "Lezhe", "Elbasan"])

actions = np.array([[1, 4], [2, -1], [3, 5], [-1, -1], [3, -1], [3, -1]]) #value in [0][1] means from state 0 (Tirana) you can go to state 1 (Durres).
# the cost for this action is in the same index in action_cost array

action_cost = np.array([[2, 7], [1, -1], [10, 5], [-1, -1], [8, -1], [2, -1]]) # in the same order as actions array

ids = IDS(0, 3, actions, action_cost, states, evaluationFunction)
print(f'Path and path cost: {ids.iterate()}')