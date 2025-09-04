from BFS import Node, BFS
import numpy as np


# Example route finding problem in albania
states = np.array(["Sibiu", "Rimniu Vilcea", "Fagaras", "Pitesta", "Bucharest"])

actions = np.array([[1, 2], [3, -1], [4, -1], [4, -1], [-1, -1]]) 

action_cost = np.array([[80, 99], [97, -1], [211, -1], [101, -1], [-1, -1]]) # in the same order as actions array

ucs = BFS(0, 4, actions, action_cost, states)
print(f'Path and path cost: {ucs.search()}')