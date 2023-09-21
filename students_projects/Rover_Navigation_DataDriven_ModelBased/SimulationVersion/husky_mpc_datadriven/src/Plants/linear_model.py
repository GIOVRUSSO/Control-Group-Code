import numpy as np
import random

"""
linear_model: A class representing a simple linear dynamic model.

This class defines a simple linear model with two state variables and methods to perform simulation steps.
"""
class linear_model:

    """
    Initialize the linear_model class with given coefficients.
    
    :param a1: Coefficient for state x1.
    :param a2: Coefficient for state x2.
    """
    def __init__(self, a1, a2):
        self.x = np.zeros(2)
        self.a1 = a1
        self.a2 = a2


    """
    Set the initial state.
    
    :param x0: Initial state vector.
    """
    def set_x0(self, x0):
        self.x = x0


    """
    Perform a simulation step for the linear model.
    
    :param u: Control input.
    :return: New state variables x1_new and x2_new.
    """
    def make_step(self, u):
        x1, x2 = self.x  # Current state
        
        # Update the state based on the linear system equations
        x1_new = self.a1 * u * 0.5 + x1
        x2_new = self.a2 * u * 0.5 + x2
        
        # Ensure state values are not negative
        x1_new = max(x1_new, 0)
        x2_new = max(x2_new, 0)
        
        # Ensure state values are not above 18
        x1_new = min(x1_new, 9.5)
        x2_new = min(x2_new, 9.5)
        
        # Update the state for the next step
        self.x = np.array([x1_new, x2_new]) 
        
        return x1_new, x2_new


#Generate Simulations for the data driven process
simulation_with_random = np.zeros((6000, 50, 2))
a1 = [1, 0, 0.5]
a2 = [0, 1, 0.5]


# Simulate for two different sets of coefficients
for index in range(3):
    rover = linear_model(a1[index], a2[index])
    for sim in range(2000):
        rover.set_x0([random.uniform(0, 9), random.uniform(0, 9)])
        for iteration in range(50):
            uk = random.uniform(-1, 1)
            xk, yk = rover.make_step(uk)
            simulation_with_random[sim+2000*index, iteration] = np.array([xk, yk])

# Save the generated simulation data to a file
np.save('2TypeSimulation.npy', simulation_with_random)

