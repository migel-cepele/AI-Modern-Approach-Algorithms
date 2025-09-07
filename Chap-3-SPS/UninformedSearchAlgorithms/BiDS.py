from Node import Node
import numpy as np

def evaluationFunction(frontier):
        return np.argmin([node.pathCost for node in frontier])


class BiDS:
    def __init__(self, start_f, goal_f, actions_f, action_costs_f, 
                 start_b, goal_b, actions_b, action_costs_b, evaluation=None):
        self.start_f = Node(start_f)
        self.goal_f = goal_f
        self.actions_f = actions_f
        self.action_costs_f = action_costs_f
        self.start_b = Node(start_b)
        self.goal_b = goal_b
        self.actions_b = actions_b
        self.action_costs_b = action_costs_b

        self.evaluationFunction = evaluation if evaluation else self.evaluationFunction

    #direction will be defined by chars 'F' and 'B'
    def search(self):
        frontierF = [self.start_f]
        frontierB = [self.start_b]
        reachedF = [self.start_f]
        reachedB = [self.start_b]
        solution = []

        while not self.terminated(solution, frontierF, frontierB):

            if(frontierF[-1].pathCost <= frontierB[-1].pathCost):
                solution = self.proceed('F', frontierF, reachedF, reachedB, solution)
            else:
                solution = self.proceed('B', frontierB, reachedB, reachedF, solution)
        solutionStates = [item.state for item in solution]

        print("Solution ", solutionStates)
        return solutionStates, self.solutionPathCost(solution)

    #check is solution contains the initial and end state
    def terminated(self, solution, frontierF, frontierB):
        if not frontierB or not frontierF:
            return True
        return (self.start_f in solution) and (self.start_b in solution) 

    # helper function to build the path
    def buildPath(self, node : Node):
        path = []
        while node is not None:
            path.append(node)
            node = node.parent
        return list(reversed(path))    
    
    # joins the paths to the node from both directions
    def joinNodes(self, dir, child : Node, reached_s2 : Node):
        if dir == 'F':
            path1 = self.buildPath(child)
            path2 = self.buildPath(reached_s2)
            path2.reverse() #so we can continue with reached_s2 node

            return path1 + path2[1:] #because reached s2 would be duplicate
        else:
            path1 = self.buildPath(reached_s2)
            path2= self.buildPath(child)
            path2.reverse()

            return path1 + path2[1:]
        
    # solution path cost start from the initial node
    def solutionPathCost(self, solution : list[Node]):
        pathCost = 0
        for i in range(len(solution) - 1):

            actions = self.actions_f[solution[i].state]
            for j in range(len(actions)):

                if actions[j] == solution[i+1].state:
                    pathCost += self.action_costs_f[solution[i].state, j]
                    break

        return pathCost

    # path cost for paths formed while searching
    def pathCost(self, solution):
        return solution[-1].pathCost if solution else float('inf')
              

    def proceed(self, dir, frontier: list[Node], reached : list[Node], reached2 : list[Node], solution : list[Node]):

        node = frontier.pop()

        children : list[Node] = self.expand(node, dir)

        for child in children:
            reached_s = next((n for n in reached if n.state == child.state), None)

            if reached_s == None or child.pathCost < reached_s.pathCost:

                if reached_s != None:
                    reached.remove(reached_s
                                   )
                reached.append(child)
                frontier.append(child)
                print("Adding state ", child.state ," to frontier and reached in direction ", dir)

                reached_s2 = next((n for n in reached2 if n.state == child.state), None)
                if reached_s2 != None:
                    solution2 = self.joinNodes(dir, child, reached_s2)

                    if self.pathCost(solution2) < self.pathCost(solution):
                        solution = solution2
        return solution

    # by default the evaluation function returns the index of the node with the lowest path cost
    def evaluationFunction(self, frontier):
        return np.argmin([node.pathCost for node in frontier])


    # nodes will be expanded depending on the direction of the search
    def expand(self, node: Node, dir):
        s = node.state
        print("Node to expand: ", s, " in dir ", dir)

        expanded_nodes = []
        actions_s = np.array([])
        action_costs_s = np.array([])  
        
        if dir == 'F':
            actions_s = self.actions_f[s]
            print("Possible actions F: ", self.actions_f[s])

            action_costs_s = self.action_costs_f[s]
            print("Action costs F: ", self.action_costs_f[s])  
        else:
            actions_s = self.actions_b[s]
            print("Possible actions B: ", self.actions_b[s])

            action_costs_s = self.action_costs_b[s]
            print("Action costs B: ", self.action_costs_b[s])  

        for i in range(len(actions_s)):
            if actions_s[i] == -1:
                continue
            cost = node.pathCost + action_costs_s[i] # get the cost of the action
            new_node = Node(actions_s[i], node, cost, node.depth + 1)
            expanded_nodes.append(new_node) # add the new node
    
        print("Expanded nodes: ", expanded_nodes)
        return expanded_nodes
    

# Example route finding problem in albania
states = np.array(["Sibiu", "Rimniu Vilcea", "Fagaras", "Pitesta", "Bucharest"])

actions_forward = np.array([[1, 2], [3, -1], [4, -1], [4, -1], [-1, -1]]) 
action_cost_forward = np.array([[80, 99], [97, -1], [211, -1], [101, -1], [-1, -1]]) # in the same order as actions array
start_forward = 0

actions_backward = np.array([[-1, -1], [0, -1], [0, -1], [1, -1], [2, 3]]) 
action_cost_backward = np.array([[-1, -1], [80, -1], [99, -1], [97, -1], [211, 101]]) # in the same order as actions array
start_backward = 4

bids = BiDS(0, 4, actions_forward, action_cost_forward, 4, 0, actions_backward, action_cost_backward)
print(f'Path and path cost: {bids.search()}')