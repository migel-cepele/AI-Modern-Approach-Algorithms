from State import State
import numpy as np

class HC:
    def __init__(self, initialState, actions, action_costs):
        self.initialState = State(initialState)
        self.actions = actions
        self.action_costs = action_costs

    
    def hillClimbing(self):
        current = self.initialState
        while True:
            neighbour = self.expand(current)

            if neighbour is None : return current.name, current.value

            if neighbour.value <= current.value:
                return current.name, current.value
            current = neighbour

    
    def expand(self, state: State):
        s = state.name
        print("State to expand: ", s)
        
        actions_s = self.actions[s]
        print("Possible actions: ", self.actions[s])

        action_costs_s = self.action_costs[s]
        print("Action costs: ", self.action_costs[s])

        expanded_states = []
        max_state = None
        max_value = -1

        for i in range(len(actions_s)):
            if actions_s[i] == -1:
                continue
            value = state.value + action_costs_s[i] # get the cost of the action
            newState = State(actions_s[i], value, state)
            
            if value > max_value:
                max_value = value
                max_state = newState

            expanded_states.append(newState) # add the new node
    
        print("Expanded states: ", expanded_states)
        print("max expanded state", max_state)
        return max_state



states = np.array(["Sibiu", "Rimniu Vilcea", "Fagaras", "Pitesti", "Bucharest"])

actions = np.array([[1, 2], [3, -1], [4, -1], [4, -1], [-1, -1]]) 

action_cost = np.array([[80, 99], [97, -1], [211, -1], [101, -1], [-1, -1]]) # in the same order as actions array

hc = HC(0, actions, action_cost)
print("Max state with value ", hc.hillClimbing()) # highest value state