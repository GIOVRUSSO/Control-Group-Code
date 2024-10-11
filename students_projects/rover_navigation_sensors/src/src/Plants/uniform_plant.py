"""
uniform_plant: A class representing a uniform plant model.

This class defines a simple uniform plant model that represents passive dynamics as uniform transitions
from one state to its neighboring states.
"""

import numpy as np

class uniform_plant:

    """
    Generate the transition matrix for the uniform plant model.
    
    :param self: The instance of the class.
    :param dim: The dimensionality of the state space.
    :return: The transition matrix representing passive dynamics.
    """
    def get_plant(self, dim):

        # Initialize the transition matrix
        passive_dynamics = np.zeros((dim, dim, dim, dim))

        # Populate transitions for adjacent states
        for row in range(dim):
            for col in range(dim):
                current_state = (row, col)  # Current state

                # Possible transitions: up, down, left, right
                possible_transitions = [(0, -1), (0, 1), (-1, 0), (1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]

                for dr, dc in possible_transitions:
                    next_row = row + dr
                    next_col = col + dc

                    # Check if the next position is within the grid
                    if 0 <= next_row < dim and 0 <= next_col < dim:
                        # Set the transition probability from current_state to next_state
                        passive_dynamics[row, col, next_row, next_col] = 1.0 / len(possible_transitions)

        # Set self-transitions probabilities to 0
        for row in range(dim):
            for col in range(dim):
                passive_dynamics[row, col, row, col] = 0

        return passive_dynamics
