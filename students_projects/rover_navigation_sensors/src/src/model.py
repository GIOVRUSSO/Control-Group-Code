import do_mpc
from casadi import *
import numpy as np
from cost_cache import *
from tf import transformations as t
from model import *

"""
Model: A class representing the dynamic model for the system.

This class defines a continuous-time model for the robot's dynamics, including state variables, control variables,
and time-varying parameters. It sets up the state equations and provides a method to retrieve the model.
"""
class Model():

    """
    Initialize the Model class by defining the dynamic model.
    
    :param self: The instance of the class.
    """
    def __init__(self):

        model_type = 'continuous'  # either 'discrete' or 'continuous'
        self.model = do_mpc.model.Model(model_type)

        # Define state variables 
        self.x = self.model.set_variable(var_type='_x', var_name='x', shape=(1, 1))
        self.y = self.model.set_variable(var_type='_x', var_name='y', shape=(1, 1))
        self.theta = self.model.set_variable(var_type='_x', var_name='theta', shape=(1, 1))

        # Define control variables
        self.v = self.model.set_variable(var_type='_u', var_name='v')
        self.w = self.model.set_variable(var_type='_u', var_name='w')

        # Define Time-varying parameters
        self.xd = self.model.set_variable(var_type='_tvp', var_name='xd')
        self.yd = self.model.set_variable(var_type='_tvp', var_name='yd')

        # Define state equations
        self.model.set_rhs('x', self.v * np.cos(self.theta))
        self.model.set_rhs('y', self.v * np.sin(self.theta))
        self.model.set_rhs('theta', self.w)
        self.model.setup()

    """
    Get the defined dynamic model.
    
    :param self: The instance of the class.
    :return: The dynamic model.
    """
    def get_model(self):
        return self.model