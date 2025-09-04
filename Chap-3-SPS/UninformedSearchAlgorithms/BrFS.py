from BFS import Node
import numpy as np

class BrFS:
    def __init__(self, s_start, goal, actions, action_costs, states):
        self.start = Node(s_start)
        self.goal = goal
        self.actions = actions
        self.action_costs = action_costs
        self.states = states

    def search(self):
        print("Breadth First Search")

        if self.start.state == self.goal:
            print('Start is the goal')
            return [self.states[self.s_start]], 0
        
        frontier = [self.start] # initialize the frontier with the start node
        reached = [self.start.state] # initialize the reached set with the start node state

        while len(frontier) > 0:
            node = frontier.pop(0) # get the first node in the frontier (FIFO)

            children = self.expand(node) # expand the node to get its children                      

            for i in range(len(children)):

                s = children[i].state
                
                if s == self.goal: # check if the node is the goal
                    print("Goal Reached: ", s)
                    return children[i].path(), children[i].pathCost  

                if children[i].state not in reached:
                    frontier.append(children[i])
                    reached.append(children[i].state)
                    print("Node was added to reached: ", children[i].state)

        return None, float('inf') # return None if the goal is not reachable
    
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
            new_node = Node(actions_s[i], node, cost)
            expanded_nodes.append(new_node) # add the new node
    
        print("Expanded nodes: ", expanded_nodes)
        return expanded_nodes



# Example route finding problem in albania

states = np.array(["Tirana", "Durres", "Kavaje", "Vlore", "Lezhe", "Elbasan"])

actions = np.array([[1, 4], [2, -1], [3, 5], [-1, -1], [3, -1], [3, -1]]) #value in [0][1] means from state 0 (Tirana) you can go to state 1 (Durres).
# the cost for this action is in the same index in action_cost array

action_cost = np.array([[1, 1], [1, -1], [1, 1], [-1, -1], [1, -1], [1, -1]]) # in the same order as actions array

brfs = BrFS(0, 3, actions, action_cost, states)
print(f'Path and path cost: {brfs.search()}')