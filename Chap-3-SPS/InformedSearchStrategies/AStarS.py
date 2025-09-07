import numpy as np
import matplotlib.pyplot as plt
from Node import Node

# for simplicity and clarity the state is represented as a number and treated as an index to the array  
class AStarS:
    def __init__(self, s_start, goal, actions, action_costs, states, evaluation, hsld):
        self.start = Node(s_start)
        self.goal = goal
        self.actions = actions
        self.action_costs = action_costs
        self.states = states
        self.evaluationFunction = evaluation if evaluation else self.evaluationFunction
        self.hsld = hsld

    # with early goal check we find another path, which might not be optimal
    # with late goal check we find a better path(currently)
    def search(self):
        print("AStarS Search")
        frontier = self.expand(self.start) # initialize the frontier with the children of the start node    
        reached = [self.start] # initialize the reached set with the start node state

        #if self.start.state == self.goal: # check if the node is the goal
         #       print("Goal Reached: ", self.start.state)
          #      return self.start.path(), self.start.pathCost

        while len(frontier) > 0:
            #frontier is a priority queue based on heuristic function
            min_index = self.evaluationFunction(frontier)
            node = frontier.pop(min_index)

            if node.state == self.goal: # check if the node is the goal
                print("Goal Reached: ", node.state)
                return node.path(), node.pathCost
      
            children : list[Node] = self.expand(node) # expand the node to get its children

            for i in range(len(children)):

                #if children[i].state == self.goal: # check if the node is the goal
                 #   print("Goal Reached: ", children[i].state)
                  #  return children[i].path(), children[i].pathCost
            
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
            print("state ", actions_s[i], " has f=g+h ", cost, self.hsld[actions_s[i]], cost + self.hsld[actions_s[i]])
            expanded_nodes.append(new_node) # add the new node
    
        print("Expanded nodes: ", expanded_nodes)
        return expanded_nodes



# Example route finding problem in romania
states = np.array([
    "Arad", "Zerind", "Oradea", "Sibiu", "Fagaras", "Rimnicu Vilcea",
    "Pitesti", "Timisoara", "Lugoj", "Mehadia", "Drobeta", "Craiova",
    "Bucharest", "Giurgiu", "Urziceni", "Hirsova", "Eforie",
    "Vaslui", "Iasi", "Neamt"
])


actions = np.array([
    [1, 3, 7, -1],          # Arad → Zerind, Sibiu, Timisoara
    [0, 2, -1, -1],         # Zerind → Arad, Oradea
    [1, 3, -1, -1],         # Oradea → Zerind, Sibiu
    [0, 2, 4, 5],           # Sibiu → Arad, Oradea, Fagaras, Rimnicu Vilcea
    [3, 12, -1, -1],        # Fagaras → Sibiu, Bucharest
    [3, 6, 11, -1],         # Rimnicu Vilcea → Sibiu, Pitesti, Craiova
    [5, 11, 12, -1],        # Pitesti → Rimnicu Vilcea, Craiova, Bucharest
    [0, 8, -1, -1],         # Timisoara → Arad, Lugoj
    [7, 9, -1, -1],         # Lugoj → Timisoara, Mehadia
    [8, 10, -1, -1],        # Mehadia → Lugoj, Drobeta
    [9, 11, -1, -1],        # Drobeta → Mehadia, Craiova
    [10, 5, 6, -1],         # Craiova → Drobeta, Rimnicu Vilcea, Pitesti
    [-1, -1, -1, -1],       # Bucharest → Fagaras, Pitesti, Giurgiu, Urziceni
    [12, -1, -1, -1],       # Giurgiu → Bucharest
    [12, 15, 17, -1],       # Urziceni → Bucharest, Hirsova, Vaslui
    [14, 16, -1, -1],       # Hirsova → Urziceni, Eforie
    [15, -1, -1, -1],       # Eforie → Hirsova
    [14, 18, -1, -1],       # Vaslui → Urziceni, Iasi
    [17, 19, -1, -1],       # Iasi → Vaslui, Neamt
    [18, -1, -1, -1],       # Neamt → Iasi
])

# corresponding costs (aligned with actions)
action_cost = np.array([
    [75, 140, 118, -1],     # Arad
    [75, 71, -1, -1],       # Zerind
    [71, 151, -1, -1],      # Oradea
    [140, 151, 99, 80],     # Sibiu
    [99, 211, -1, -1],      # Fagaras
    [80, 97, 146, -1],      # Rimnicu Vilcea
    [97, 138, 101, -1],     # Pitesti
    [118, 111, -1, -1],     # Timisoara
    [111, 70, -1, -1],      # Lugoj
    [70, 75, -1, -1],       # Mehadia
    [75, 120, -1, -1],      # Drobeta
    [120, 146, 138, -1],    # Craiova
    [-1, -1, -1, -1],       # Bucharest
    [90, -1, -1, -1],       # Giurgiu
    [85, 98, 142, -1],      # Urziceni
    [98, 86, -1, -1],       # Hirsova
    [86, -1, -1, -1],       # Eforie
    [142, 92, -1, -1],      # Vaslui
    [92, 87, -1, -1],       # Iasi
    [87, -1, -1, -1],       # Neamt
])

# straight line distances to the destination. The index represents the distance from the state
# with that name to the destination Bucharest
hsld = np.array([
    366,  # 0 Arad
    374,  # 1 Zerind
    380,  # 2 Oradea
    253,  # 3 Sibiu
    176,  # 4 Fagaras
    193,  # 5 Rimnicu Vilcea
    100,  # 6 Pitesti
    329,  # 7 Timisoara
    244,  # 8 Lugoj
    241,  # 9 Mehadia
    242,  # 10 Drobeta
    160,  # 11 Craiova
    0,    # 12 Bucharest
    77,   # 13 Giurgiu
    80,   # 14 Urziceni
    151,  # 15 Hirsova
    161,  # 16 Eforie
    199,  # 17 Vaslui
    226,  # 18 Iasi
    234   # 19 Neamt
])

# evaluation function returns the node with min heuristic function
def evaluation(frontier):
    return np.argmin([node.h + node.pathCost for node in frontier])


aStarS = AStarS(0, 12, actions, action_cost, states, evaluation, hsld)
print(f'Path and path cost: {aStarS.search()}')

