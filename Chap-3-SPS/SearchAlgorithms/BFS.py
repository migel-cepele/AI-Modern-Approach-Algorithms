import numpy as np
import matplotlib.pyplot as plt
from Node import Node

# for simplicity and clarity the state is represented as a number and treated as an index to the array  
class BFS:
    def __init__(self, s_start, goal, actions, action_costs, states, evaluation=None):
        self.start = Node(s_start)
        self.goal = goal
        self.actions = actions
        self.action_costs = action_costs
        self.states = states
        self.evaluationFunction = evaluation if evaluation else self.evaluationFunction

    def search(self):
        print("BFS Search")
        frontier = self.expand(self.start) # initialize the frontier with the children of the start node    
        reached = [self.start] # initialize the reached set with the start node state

        while len(frontier) > 0:
            #frontier is a priority queue based on path cost
            min_index = self.evaluationFunction(frontier)
            node = frontier.pop(min_index)

            if node.state == self.goal: # check if the node is the goal
                print("Goal Reached: ", node.state)
                return node.path(), node.pathCost
            
            children = self.expand(node) # expand the node to get its children

            for i in range(len(children)):

                s = children[i].state
                
                reached_s = next((n for n in reached if n.state == s), None) # check if the state is in the reached set

                if reached_s == None or children[i].pathCost < reached_s.pathCost:
                    frontier.append(children[i])
                    # remove and add another better node to reached set if it exists
                    if reached_s != None:
                        reached.remove(reached_s)

                    reached.append(children[i])
                    print("Node was added to reached: ", children[i].state)

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



# Example route finding problem in albania
states = np.array(["Tirana", "Durres", "Kavaje", "Vlore", "Lezhe", "Elbasan"])

actions = np.array([[1, 4], [2, -1], [3, 5], [-1, -1], [3, -1], [3, -1]]) #value in [0][1] means from state 0 (Tirana) you can go to state 1 (Durres).
# the cost for this action is in the same index in action_cost array

action_cost = np.array([[2, 7], [1, -1], [4, 5], [-1, -1], [8, -1], [2, -1]]) # in the same order as actions array

bfs = BFS(0, 3, actions, action_cost, states)
print(f'Path and path cost: {bfs.search()}')

