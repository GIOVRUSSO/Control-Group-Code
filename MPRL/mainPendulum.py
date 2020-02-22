from qlearning import QLearning
from numpy import pi, cos, sin, exp
import random
import time

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from simulation import Simulator
import numpy as np

class InvertedPendulumBalancer:
    def __init__(self):
        self.dt = 0.01
        self.max_force = 125
        self.step_n =  int(450) # number of steps
        self.simulator = Simulator()
        self.Ctheta = -1
        self.Ktheta = 0.05
        self.Cx = 0.5
        self.Kx = 10    
    def run(self):

        for j in range(1):
            random.seed(200) #sets initial value of random number generator
            self.controller = QLearning( max_theta=2*pi, max_dtheta=30, max_x=60, max_dx=60, n_x=10, n_dx=10, n_theta=6, n_dtheta=20, n_action=2, is_fuzzy=True )
            
            state = [-0.2, 0, pi-0.15, 0] # state = [linear position, linear velocity, angular position, angular velocity]
            states = [] # will hold each individual state
            t = 0
            plot_n = 2 # number of iterations before plotting
            plot_resolution = 1 # number of iterations before adding state to list
            
            """variables needed to plot graphs """
            reward = 0
            states = []
            last_stable = 0
            survival_times = []
            survival_time = 0
            thetaplot = pi
            thetaplots=[]
            lines = []
            rewards = []
            constraint1 =[]
            constraint2=[]
            
            """ if want to visualise cart pole uncomment this part"""
#            plt.rc_context({'axes.edgecolor':'orange', 'xtick.color':'red', 'ytick.color':'green', 'figure.facecolor':'white', 'axes.linewidth': 2})
#
#            plt.ion()
#            mng = plt.get_current_fig_manager()
#            mng.resize(*mng.window.maxsize())
#
#            theta_ax = plt.subplot2grid((4,3), (2,0), colspan=3)
#            cart_ax = plt.subplot2grid((2,3), (0,0), colspan=3)


        
            for i in range(self.step_n):
                state[2] += (random.random() - 0.5) * 0.005 # state[2] is the angular position
             
                prev_state = state
                if i % plot_resolution == 0:
                    states.append(state)
                    theta = state[2]
                    thetaplot = (180/pi)*theta
                    thetaplots.append(thetaplot)
                    constraint1.append(135)
                    constraint2.append(225)
                    
                if theta <= pi/2 or theta >= (3*pi)/2:
                    state = self.simulator.random_state(state) # reset if pendulum about to fall to the bottom
                    
                if state[2] <= (pi - pi/4) or state[2] >= (pi + pi/4):
                    survival_time = survival_time + 1

                survival_times.append(survival_time)
                    
                q_state = [state[0], state[1], state[2], state[3]] # define q state
                action = self.controller.action(q_state) # get an action by passing state into controller
                force = self.max_force * action[1] # apply force to that action
                state = self.simulator.simulate_step(state, force, self.dt) # get next state based on that action
                next_q_state = [state[0], state[1], state[2] + pi, state[3]] # set next sate

              # reward is 1 unless angle goes beyond limits then reward gets more negative for angles further away from verticle
                if abs(pi - state[2]) >= self.Ktheta: # if pi - state2 has big difference means pendulum more away from vertical therefore bigger the diff bigger 
                                                      #  the penalty i.e. penalty reduces closer pendulum to vertical
                    reward = self.Ctheta * ( abs(pi-state[2])) 
                elif abs(pi - state[2]) <=self.Ktheta:
                    reward = 1

                if abs(state[0]) >= self.Kx:
                    reward -= abs(state[0])**self.Cx # if cart moves out of range reward is penalised
                
                
                self.controller.update(q_state, action[0], next_q_state, reward,0.7, 0.7) # update the controller
                rewards.append(reward)

""" uncomment this part if want to visualise cart pole"""
                # visualising                                                  
#                if i > 0 and i % (plot_n - 1) == 0:
#
#                   theta_ax.plot([s[2] for s in states], color='r')
#
#                   cart_ax.patches = []
#                   cart_ax.artists = []
#                   cart_ax.lines = []
#                   cart_width = 20
#                   cart_height = 2.5
#                   factor = 2
#                   r = 1
#                   cart_ax.axis([-factor * cart_width, factor * cart_width, 0, factor * cart_width])
#                   p_fancy = FancyBboxPatch((-cart_width / 2 + state[0] , 2.5 * r),
#                                            abs(cart_width), abs(cart_height),
#                                            boxstyle="round,pad=0.5",
#                                            fc=(0, 1.0, 0),
#                                            ec=(1., 0.5, 1.))
#                   cart_ax.add_patch(p_fancy)
#                   circle1 = plt.Circle((-cart_width / 4 + state[0] , 2 * r), r, color='b')
#                   circle2 = plt.Circle((cart_width / 4 + state[0] , 2 * r), r, color='b')
#                   cart_ax.add_artist(circle1)
#                   cart_ax.add_artist(circle2)
#                   
#                   L = 6 * cart_height
#                   L_discount = (L + L * sin(pi/2 - state[2]) ** 2)
#                   cart_ax.plot([state[0], state[0] - L_discount * cos(pi/2 - state[2])], 
#                                 [1.5 * cart_height, 1.5 * cart_height -  L_discount * sin(pi/2 - state[2])], 
#                                 color='b', 
#                   solid_capstyle="round",
#                   linewidth=2)
#               
#                   plt.pause(0.000001)
#                   

        return rewards, survival_times, thetaplots, constraint1, constraint2

balancer = InvertedPendulumBalancer()

rewards, survival_times, thetaplots, constraint1, constraint2=balancer.run()

plt.plot(survival_times)
#plt.plot(constraint1, label='Constraint at 135 degrees')
#plt.plot(constraint2, label='Constraint at 225 degrees')
#plt.legend(loc='lower right')
plt.xlabel('Time step')
plt.ylabel('No. of violations')
plt.title('Purely Q-learning')
plt.show()




