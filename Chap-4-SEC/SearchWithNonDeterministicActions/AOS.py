from Node import Node
import numpy as np


class AOS:
    def __init__(self, initial_state, goal_state, actions, action_costs):
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.actions = actions
        self.action_costs = action_costs

    def is_cycle(self, node : Node):
        state = node.state
        node_parent = node.parent
        while node_parent:
            if state == node_parent.state:
                return True
            node_parent = node_parent.parent
        return False
    
    def and_or_search(self):
        path = []
        return self.or_search(self, self.initial_state, path)

    def or_search(self, node : Node, path):
        if self.goal_state == node.state:
            print("goal state is found", node.state)
            return path
        
        if self.is_cycle(node):
            print("For state ", node.state, " is formed a cycle in the path above")
            return None
        
        result_nodes = self.expand(node)
        plan = self.and_search(self, )
        


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


# Example route finding problem in romania, but suppose now it is a nondeterministic environment
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