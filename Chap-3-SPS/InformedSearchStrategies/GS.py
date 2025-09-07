import numpy as np
import matplotlib.pyplot as plt
from Node import Node

# for simplicity and clarity the state is represented as a number and treated as an index to the array  
class GS:
    def __init__(self, s_start, goal, actions, action_costs, states, evaluation, hsld):
        self.start = Node(s_start)
        self.goal = goal
        self.actions = actions
        self.action_costs = action_costs
        self.states = states
        self.evaluationFunction = evaluation if evaluation else self.evaluationFunction
        self.hsld = hsld

    def search(self):
        print("GS Search")
        frontier = self.expand(self.start) # initialize the frontier with the children of the start node    
        reached = [self.start] # initialize the reached set with the start node state

        if self.start.state == self.goal: # check if the node is the goal
                print("Goal Reached: ", self.start.state)
                return self.start.path(), self.start.pathCost

        while len(frontier) > 0:
            #frontier is a priority queue based on heuristic function
            min_index = self.evaluationFunction(frontier)
            node = frontier.pop(min_index)
      
            children : list[Node] = self.expand(node) # expand the node to get its children

            for i in range(len(children)):

                if children[i].state == self.goal: # check if the node is the goal
                    print("Goal Reached: ", children[i].state)
                    return children[i].path(), children[i].pathCost
            
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
            new_node = Node(actions_s[i], node, cost, node.depth + 1, self.hsld[actions_s[i]])
            print("state ", actions_s[i], " has hsld ", self.hsld[actions_s[i]])
            expanded_nodes.append(new_node) # add the new node
    
        print("Expanded nodes: ", expanded_nodes)
        return expanded_nodes



# Example route finding problem in romania
states = np.array(["Sibiu", "Rimniu Vilcea", "Fagaras", "Pitesti", "Bucharest"])

actions = np.array([[1, 2], [3, -1], [4, -1], [4, -1], [-1, -1]]) 

action_cost = np.array([[80, 99], [97, -1], [211, -1], [101, -1], [-1, -1]]) # in the same order as actions array

# straight line distances to the destination. The index represents the distance from the state
# with that name to the destination Bucharest
hsld = np.array([253, 193, 176, 100, 0])

# evaluation function returns the node with min heuristic function
def evaluation(frontier):
    return np.argmin([node.h for node in frontier])


gs = GS(0, 4, actions, action_cost, states, evaluation, hsld)
print(f'Path and path cost: {gs.search()}')

