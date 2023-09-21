import rospy
from geometry_msgs.msg import Twist
from tf import transformations as t
import numpy as np
from utility import *
from cost_cache import *
import do_mpc
from casadi import *
import numpy as np
from cost_cache import *
from model import *

"""
ControllerMPC: A class implementing a Model Predictive Control (MPC) controller for a robot's motion.
"""


class ControllerMPC:

    """
    A method to calculate the time-varying parameters (TVP) for the MPC controller.
    
    :param self: The instance of the class.
    :return: The time-varying parameters for the controller.
    """
    def tvp_fun(self, time):

        print(self.index)
        if self.index >= 45:  #the result can't reach a number of waypoint after the simulation dimension
             self.index = 44

        for k in range(21):
                x, y = self.cache.next_target(self.index)
                self.tvp_template['_tvp', k, 'xd'] = x
                self.tvp_template['_tvp', k, 'yd'] = y

        return self.tvp_template

    """
    Initialize the MPC controller with initial state.
    
    :param self: The instance of the class.
    :param init_state: The initial state of the robot.
    """
    def __init__(self, init_state):

        self.cache = CostCache()
        self.index = 0

        print("prova")
        self.model = Model().get_model()

        print("prova")

        #mpc controller INIT
        setup_mpc = {
            'n_horizon': 20,
            't_step': 0.1,
            'n_robust': 1,
            'store_full_solution': True,
            'supress_ipopt_output': True
        }

        self.mpc = do_mpc.controller.MPC(self.model)
        self.mpc.set_param(**setup_mpc)

        self.set_cost_function()
        self.set_bounds()

        self.tvp_template = self.mpc.get_tvp_template()
        self.mpc.set_tvp_fun(self.tvp_fun)

        self.mpc.setup()
        
        # Set initial state for simulations
        x0 = np.array(init_state).reshape(-1, 1)
        self.mpc.x0 = x0
        self.mpc.set_initial_guess()


    """
    Update the controller and apply the calculated control input to the robot.
    
    :param self: The instance of the class.
    """
    def update(self, target, states):

        # Get the robot's current states (position and orientation)
        
        print(states)
        states = np.array(states).reshape(-1,1)

        #Implement disturbance on the model
        #states_noise = np.array(add_noise_to_states(states)).reshape(-1,1)

        # Perform MPC step to get the control input
        u = self.mpc.make_step(states)

        #Change reference if the previous target got
        if u[0] <= 0.5 and u[1] <= 0.5:
            self.index+=1

        if abs(states[0]-target[0])<=0.2 and abs(states[1]-target[1])<=0.2:
            return states[0], states[1], 1

        #self.cache.set_actual_state(states[0], states[1])
        
        return u, states[0], states[1], 0


    """ METHOD FOR THE __init__ and update Method (utility)"""


    """
    Set the bounds for states and control inputs for the MPC controller.
    
    :param self: The instance of the class.
    """
    def set_bounds(self):
        # Set lower bounds on states
        self.mpc.bounds['lower', '_x', 'x'] = 0
        self.mpc.bounds['lower', '_x', 'y'] = 0

        self.mpc.bounds['upper', '_x', 'x'] = 18
        self.mpc.bounds['upper', '_x', 'y'] = 18

        self.mpc.bounds['lower', '_x', 'theta'] = -np.pi
        self.mpc.bounds['upper', '_x', 'theta'] = np.pi

        # Set lower bounds on inputs
        self.mpc.bounds['lower', '_u', 'v'] = -0.6
        self.mpc.bounds['lower', '_u', 'w'] = -0.3

        # Set upper bounds on inputs
        self.mpc.bounds['upper', '_u', 'v'] = 0.6
        self.mpc.bounds['upper', '_u', 'w'] = 0.3


    """
    Set the cost function for the MPC controller.
    
    :param self: The instance of the class.
    """
    def set_cost_function(self):       

        mterm = 2*(self.model.x['x'] - self.model.tvp['xd'])**2 + 2*(self.model.x['y'] - self.model.tvp['yd'])**2 
        lterm = mterm + 1/2*self.model.u['v']**2 + 1/2*self.model.u['w']**2 

        self.mpc.set_objective(mterm=mterm, lterm=lterm)
