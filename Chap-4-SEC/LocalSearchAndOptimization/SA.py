import random
from State import State
import numpy as np

class SA:
    def __init__(self, initial_state, actions, action_costs):
        self.initial_state = State(initial_state)
        self.actions = actions
        self.action_costs = action_costs

    def exp_schedule(self, k = 20, lam = 0.005, limit = 5): #limit is lower here because here are a few states
        return lambda t: (k * np.exp(-lam * t) if t < limit else 0)
    
    def probability(self, p):
        return p > random.uniform(0.0, 1.0)

    def simulated_annealing(self):
        current = self.initial_state
        t = 1
        while True:
            T = self.exp_schedule()(t)

            if T == 0:
                return current.name, current.value
            
            neighbours : list[State] = self.expand(current)
            if len(neighbours) > 0:
                rand_int = np.random.randint(0, len(neighbours))
                next = neighbours[rand_int]

            if not next:
                return current.name, current.value
            
            delta_E = current.value - next.value
            if delta_E > 0 or self.probability(np.exp(-delta_E / T)):
                current = next

            t+=1
            
    


    def expand(self, state: State):
        s = state.name
        print("State to expand: ", s)
        
        actions_s = self.actions[s]
        print("Possible actions: ", self.actions[s])

        action_costs_s = self.action_costs[s]
        print("Action costs: ", self.action_costs[s])

        expanded_states = []

        for i in range(len(actions_s)):
            if actions_s[i] == -1:
                continue
            value = state.value + action_costs_s[i] # get the cost of the action
            newState = State(actions_s[i], value, state)
        

            expanded_states.append(newState) # add the new node
    
        print("Expanded states: ", expanded_states)
        return expanded_states

states = np.array(["Sibiu", "Rimniu Vilcea", "Fagaras", "Pitesti", "Bucharest"])

actions = np.array([[1, 2], [3, -1], [4, -1], [4, -1], [-1, -1]]) 

action_costs = np.array([[80, 99], [97, -1], [211, -1], [101, -1], [-1, -1]]) # in the same order as actions array

sa = SA(0, actions, action_costs)
print("Max state with value ", sa.simulated_annealing()) # highest value state