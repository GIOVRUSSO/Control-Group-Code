# -*- coding: utf-8 -*-
"""
Created on Sat Mar 30 18:27:45 2019

@author: megha
"""

import random
from math import pi
import numpy as np

class QLearning:
    def __init__(self, ball_x=80, ball_y=80, ai_pos_y=80, v_x=80, v_y=80, n_action=3):

        self.init_constants()
        self.ball_values_x = ball_x
        self.ball_values_y = ball_y
        self.v_x=v_x
        self.v_y=v_y
        self.ai_pos_y = ai_pos_y
        self.n_action = n_action
        
        shape = (ball_x, ball_y, ai_pos_y, v_x, v_y, n_action )
                
        self.Q = self.initial_Q*np.ones(shape, dtype=float)


    def init_constants(self):
        self.initial_Q = 0

    def action(self, state):
        
        actions = self.Q[tuple(list(state))]
        actions = [ (i, actions[i]) for i in range(len(actions)) ]       
        max_action = max(actions, key=lambda x: x[1])
        return max_action[0] # give index of the action selected

    def update(self, s, a, next_s, r, gamma = 0.9, alpha=0.9):             
        max_action = max( list(self.Q[ tuple(next_s) ]) ) # from the list of actions in self.Q[input] select max
        self.Q[ tuple( list(s) + [a] ) ] = (1-alpha)*self.Q[ tuple( list(s) + [a] ) ]  + alpha*( r + gamma * max_action)
	

        
        
