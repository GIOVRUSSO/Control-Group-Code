# Agent class to organize data of controlled cars
class Agent():
    def __init__(self, edge_start, id_goal):
        self.edge_start = edge_start #Edge from which the vehicle starts their travel
        self.id_goal = id_goal #Index of the behavior corresponding to the goal
