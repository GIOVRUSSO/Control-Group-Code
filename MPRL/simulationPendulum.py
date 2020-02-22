from math import sin, cos, pi
import random

class Simulator:
    def simulate_step(self, state, F, dt):
        x, x_dot, theta, theta_dot = state

        m = 0.209 # mass of the pole
        M = 0.711  # mass of the cart WORKS BEST FOR 0.5 AND 5 WHEN TESTED
        L = 0.326  # half pole length
        K = 0.006 # mass moment of inertia of pendulum
        g = 9.81 # acceleration due to gravity
        b = 0.1 # friction constant

        A = L * sin(theta)
        B = L * cos(theta)
        
        D = g*sin(theta) + cos(theta)*((-F - m*A*theta_dot**2)/(M+m)) - (b*theta_dot)/(m*L)
        C = L*(4/3 - (m*cos(theta)**2)/(M+m))
        theta_dot_dot = D/C  
        x_dot_dot =  (F + m * A * theta_dot**2 - theta_dot_dot * B * m ) / (M + m)
        x_dot = state[1] + x_dot_dot * dt
        x = state[0] + x_dot * dt + x_dot_dot * dt * dt / 2

        theta_dot = state[3] + theta_dot_dot * dt
        theta = state[2] + theta_dot * dt + theta_dot_dot * dt * dt / 2

        return [ x, x_dot, theta, theta_dot ]

    def random_state(self, state):
        state[0] = 0
        state[1] = 0
        state[2] = pi - (random.random() - 0.5) * 0.3 # adding random noise
        state[3] = 0

        return state