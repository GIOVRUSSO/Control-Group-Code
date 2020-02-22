import random
from math import pi
import numpy as np

class QLearning:
    def __init__(self, max_theta, max_dtheta, max_x, max_dx, n_theta, n_dtheta, n_x, n_dx, n_action, is_fuzzy=False):
        self.init_constants()

        self.max_theta = max_theta
        self.max_dtheta = max_dtheta
        self.max_x = max_x
        self.max_dx = max_dx
        self.n_action = n_action
        self.n_theta = n_theta
        self.n_dheta = n_dtheta
        self.n_x = n_x
        self.n_dx = n_dx
        self.n_action = n_action
        
        shape = ( n_x, n_dx, n_theta, n_dtheta, n_action )
                
        self.Q = self.initial_Q * np.ones(shape, dtype=float)
        
                      
    def init_constants(self):
        self.initial_Q = 0
     
    def normalize_state(self, state):
        third = int( (state[2]) / ( 2.0 * self.max_theta ) * self.n_theta )
        fourth = int( (state[3] + self.max_dtheta) / ( 2.0 * self.max_dtheta ) * self.n_dheta )
        first = int( (state[0] + self.max_x) / ( 2.0 * self.max_x ) * self.n_x )
        second = int( (state[1] + self.max_dx) / ( 2.0 * self.max_dx ) * self.n_dx )
        return ( first, second, third, fourth  )

        
    def denormalize_action(self, action_index):
        if action_index == 0:
            return -1
        else:
            return 1
 """ uncomment this section if want to execute actions using purely q learning and comment out the other action function"""       
#    def action(self, state, k = 3):
#        state = self.normalize_state(state)
#        actions = self.Q[state]
#        actions = [ (i, actions[i]) for i in range(len(actions)) ]
#        max_action = max(actions, key=lambda x: x[1])
#        return max_action[0], self.denormalize_action(max_action[0])
 
 """ MPCaRL action function, if do not want to use comment out and use above action function for pure q learning"""            
    def action(self, state, k = 3):
        #MPC control
        if state[2] <= (pi - pi/4):
            return 0, -1 # 0 is the index of left action and -1 signifies to go left
        elif state[2] >= (pi + pi/4):
            return 1, 1 # 1 is the index of right action and 1 signifies to go right
        #RL control
        else:
            state = self.normalize_state(state)
            actions = self.Q[state]
            actions = [ (i, actions[i]) for i in range(len(actions)) ]
            max_action = max(actions, key=lambda x: x[1])
            return max_action[0], self.denormalize_action(max_action[0])
             
        
      
    def update(self, s, a, next_s, r, gamma = 0.7, alpha=0.5):
            
        s = self.normalize_state(s)
        next_s = self.normalize_state(next_s)
        max_action = max( list(self.Q[ tuple(next_s) ]) ) # from the list of actions in self.Q[input] select max
        self.Q[ tuple( list(s) + [a] ) ] = (1-alpha)*self.Q[ tuple( list(s) + [a] ) ]  + alpha*( r + gamma * max_action) #update action value using update equation
        



        
