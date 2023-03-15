import numpy as np


class Agent():
    def __init__(self, add_path, edge_start, id_goal):
        self.edge_start = edge_start #Starting road link
        self.id_goal = id_goal #Index of the agent's target behavior
