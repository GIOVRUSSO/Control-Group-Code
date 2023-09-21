import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from math import log
from obstacle import *
import rospy
import time
from Plants.uniform_plant import *
from Plants.linearized_plant import *
from Plants.trajectory_based_plant import *
plt.rcParams.update({'font.size': 20})

"""
ControllerKLC: A class implementing a Kinematic Linearization Controller (KLC) for robot motion planning.

This class defines methods to initialize the controller and update its behavior based on robot states.
"""
class ControllerKLC:

    """
    Update the controller's behavior based on the current state.
    
    :param self: The instance of the class.
    :return: Lists containing mean x position, mean y position, and time.
    """
    def __init__(self, goal, mode):

        self.cache = CostCache()

        self.mode = mode
        rospy.init_node('husky', anonymous=True)

        # Get the trasformation between odom and world
        self.init_position = get_position()
        self.cache.set_T(self.init_position)

        self.obstacles = Obstacle()

        #Target definition
        self.xd = goal[0]
        self.yd = goal[1]
        
        #Dimensions of the variables
        self.zdim = 2

        #Minimum values
        self.zmin = [0, 0] 

        #Discretization steps
        self.zstep = [0.25, 0.25]

        #Amount of discrete bins
        self.zdiscr = [40, 40]

        #Number of iterations for the simulations
        self.zsim = 15

        #Duration of the simulation
        self.duration = 30


        self.passive_dynamics = np.zeros((self.zdiscr[0], self.zdiscr[0], self.zdiscr[0], self.zdiscr[0]))

        if mode == 0:
            self.passive_dynamics = uniform_plant().get_plant(self.zdiscr[0])
        elif mode == 1:
            self.passive_dynamics = linearized_plant().get_plant(2)
        elif mode == 2:
            self.passive_dynamics = trajectory_based_plant().get_plant(2, uniform_plant().get_plant(self.zdiscr[0]))       


        self.stateVect = np.zeros((self.zdiscr[0]**2, 2))

        for i in range(self.zdiscr[0]):
            #Enumerate the states from 1 to 36^2. Here we explicitly build the enumeration to later build the Q matrix and for convenience
            for j in range(self.zdiscr[0]):
                # Compute the angle and speed values for the current state
                x = (i)*self.zstep[0]
                y = (j)*self.zstep[0]
                # Calculate the index of the current state in the state vector
                ind = i*self.zdiscr[0] + j

                # Assign the angle and speed values to the state vector
                self.stateVect[ind] = [x, y] 

        self.diagMinusQ = np.zeros((self.zdiscr[0]**2, self.zdiscr[0]**2)) # q

        

        for i in range(self.zdiscr[0]**2):
            #Build the diagonal matrix with the exponential of the opposite of the cost
            self.diagMinusQ[i,i] = np.exp(-self.cost(self.stateVect[i]))

        heatmap = np.zeros((self.zdiscr[0], self.zdiscr[1]))
        
        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[1]):
                #Build the diagonal matrix with the exponential of the opposite of the cost
                state = self.stateVect[i*self.zdiscr[0] + j]
                print(self.cost(state))
                heatmap[j, i] = self.cost(state)
        

        # Plot heatmap of the cost
        """plt.figure(figsize=(10, 10))
        plt.imshow(heatmap, cmap='viridis', interpolation='nearest', origin='lower')
        plt.colorbar(label='State cost')
        
        plt.title('State cost heatmap', fontsize=20)
        plt.xlabel('x', fontsize=20)
        plt.ylabel('y', fontsize=20)
        plt.show()
"""

        self.Prob = np.zeros((self.zdiscr[0]**2, self.zdiscr[0]**2))

        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[0]):
                pf = self.passive_dynamics[i,j]
                ind1 = i*self.zdiscr[0] + j
                self.Prob[ind1] = self.unravelPF(pf)

        self.z = np.array((self.zdiscr[0]**2))

        #Compute the execution time
        start_time = time.time()
        zmp = self.powerMethod(self.diagMinusQ@self.Prob, self.zdiscr[0]**2) 
        #print(zmp)
        end_time = time.time()
        elapsed_time = end_time - start_time

        zmpheat = np.zeros((self.zdiscr[0], self.zdiscr[1]))
        
        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[1]):
                #Build the diagonal matrix with the exponential of the opposite of the cost
                zmpheat[j, i] = zmp[i*self.zdiscr[0] + j]
        

        # Heatmap desiderability function
        """plt.figure(figsize=(12, 12))
        plt.imshow(zmpheat, cmap='viridis', interpolation='nearest', origin='lower')
        plt.colorbar(label='Desiderability value')
        
        plt.title('Desiderability function heatmap with Power Method', fontsize=20)
        plt.xlabel('x', fontsize=20)
        plt.ylabel('y', fontsize=20)
        plt.show()
"""


        print(f"Execution time: {elapsed_time} s")
        
        #Compute the execution time
        start_time = time.time()
        self.dynamic_programming() 
        end_time = time.time()
        elapsed_time = end_time - start_time

        print(f"Execution time: {elapsed_time} s")

        zdpheat = np.zeros((self.zdiscr[0], self.zdiscr[1]))
        
        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[1]):
                #Build the diagonal matrix with the exponential of the opposite of the cost
                zdpheat[j, i] = self.z[i*self.zdiscr[0] + j]
        

        # heatmap plot
        """plt.figure(figsize=(12, 12))
        plt.imshow(zdpheat, cmap='viridis', interpolation='nearest', origin='lower')
        plt.colorbar(label='Desiderability value')
        
        plt.title('Desiderability function heatmap with dynamic programming', fontsize=20)
        plt.xlabel('x', fontsize=20)
        plt.ylabel('y', fontsize=20)
        plt.show()"""

    

    """
	Dynamic Programming Algorithm
	    
	:param max_iterations: Maximum number of iterations for the algorithm (default is 100).
	:param convergence_threshold: Threshold for convergence (default is 1e-6).
	:return: None
	"""
    def dynamic_programming(self, max_iterations=100, convergence_threshold=1e-6):
        u = self.passive_dynamics
        p = self.passive_dynamics
        v = np.zeros((self.zdiscr[0], self.zdiscr[1]))

        for iteration in range(max_iterations):
            prev_v = np.copy(v)  # Make a copy of the previous v

            for x in range(self.zdiscr[0]):
                for y in range(self.zdiscr[1]):
                    # compute the l(x,u) term
                    current_state = x * self.zdiscr[0] + y

                    q = self.cost(self.stateVect[current_state])

                    logvalue = np.nan_to_num(np.log(u[x, y] / (p[x, y])), nan=0)

                    dkl = np.sum(u[x, y] * logvalue)

                    l = q + dkl

                    # compute the v term
                    v[x, y] = l + np.sum(u[x, y] * v)

            for x in range(self.zdiscr[0]):
                for y in range(self.zdiscr[1]):
                    u[x, y] = (p[x, y] * np.exp(-v)) / (np.sum(p[x, y] * np.exp(-v)))

            # Calculate the change in v
            delta_v = np.max(np.abs(prev_v - v))

            if delta_v < convergence_threshold:
                print(f"Converged after {iteration + 1} iterations.")
                break

        self.z = np.exp(-self.unravelPF(v))



    """
    Update the controller's behavior based on the current state.
    
    :param self: The instance of the class.
    :return: Lists containing mean x position, mean y position, and time.
    """
    def update(self):
        fullH = np.zeros((self.zsim,self.duration))
        fullHv = np.zeros((self.zsim,self.duration))
        nSteps = self.duration

        for j in range(self.zsim): #Perform zsim simulations
            hist = [[0,0]]*nSteps
            state = [0, 0] #Initialize the rover state 
            for i in range(nSteps): #For each step
                #print("Step #" + str(i))
                hist[i]=state #Log the state
                state = self.loop(state) #Sample the new state
            fullH[j] = [x[0] for x in hist]
            fullHv[j] = [x[1] for x in hist]

        meanx = [0]*self.duration #Get the means and stds for plotting
        stdsx = [0]*self.duration
        for i in range(self.duration):
            meanx[i] = np.mean(fullH[:,i])
            stdsx[i] = np.std(fullH[:,i])

        meany = [0]*self.duration #Get the means and stds for plotting
        stdsy = [0]*self.duration
        for i in range(self.duration):
            meany[i] = np.mean(fullHv[:,i])
            stdsy[i] = np.std(fullHv[:,i])

        time = np.array([time for time in range(self.duration)])

        meanx = [0]*self.duration #Get the means and stds for plotting
        stdsx = [0]*self.duration
        for i in range(self.duration):
            meanx[i] = np.mean(fullH[:,i])
            stdsx[i] = np.std(fullH[:,i])

        return [meanx, meany, time, stdsx, stdsy]

    
    # Utility methods for init and update methods
    """
    Discretize the continuous state variables.
    
    :param Z: The continuous state variables.
    :param Zdim: The dimensionality of the variables.
    :param Zmin: The minimum values for each dimension.
    :param Zstep: The discretization steps for each dimension.
    :return: The discretized indices.
    """
    def discretize(self, Z, Zdim, Zmin, Zstep):
        res = [0]*Zdim #n-dimensional index
        for i in range(Zdim): #For each dimension
            elt = Z[i] #Extract the i-th element
            ind = int((elt - Zmin[i])//Zstep[i]) #Discretize
            res[i] = ind
        return(tuple(res)) #Return as tuple for array indexing
    
    
    """
    Calculate the cost of a given state.
    
    :param state: The state to calculate the cost for.
    :return: The calculated cost.
    """
    def cost(self, state):
        k = 30
        sx = 0.7
        sy = 0.7
        q1 = 0.2
        q2 = 0.2
        obsTerm = 0

        for obs in self.obstacles.get_obs():
            xterm = ((state[0] - obs[0]) / sx) ** 2
            yterm = ((state[1] - obs[1]) / sy) ** 2
            obsTerm += k * np.exp(-0.5 * (xterm + yterm))


        return q1*(state[0] - self.xd) ** 2 + q2*(state[1] - self.yd) ** 2 + obsTerm
    
    
    """
    Unravel a 2D passive dynamics array into a 1D array.
    
    :param pf: The 2D passive dynamics array.
    :return: The unraveled 1D array.
    """
    def unravelPF(self, pf):
    
        res = np.zeros(self.zdiscr[0]**2)
        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[0]):
                res[i*self.zdiscr[0]+j] = pf[i][j]
        return(res)
    
    
    """
    Perform the power method for eigenvalue estimation.
    
    :param mat: The matrix for eigenvalue estimation.
    :param dim: The dimensionality of the matrix.
    :return: The estimated eigenvector.
    """
    def powerMethod(self, mat, dim, epsilon=1e-6):
        vect = np.ones(dim)
        nrm = np.linalg.norm(vect)
        
        for i in range(self.zsim):
            vect = mat.dot(vect)
            nrm = np.linalg.norm(vect)
            vect = vect / nrm
            print(nrm)
            
        return vect

    
    """
    Perform the power method for eigenvalue estimation.
    
    :param mat: The matrix for eigenvalue estimation.
    :param dim: The dimensionality of the matrix.
    :return: The estimated eigenvector.
    """
    def loop(self, x):
    
        ind = self.discretize(x,  self.zdim, self.zmin, self.zstep) #Discretize the state
        pf = self.passive_dynamics[ind[0],ind[1]] #Get the pf corresponding to the passive dynamics
        pf_1D = self.unravelPF(pf) #Unravel it
        pf_weighted = pf_1D*self.z #Calculate the actual transition pf using z and the passive dynamics
        S = np.sum(pf_weighted) #Normalize
        pf_weighted = pf_weighted/S #probabilities contain NaN ERRORE SPESSO USCITO FUORI forse perché non si riesce a minimizzare la funzione di costo a causa di qualche limite raggiunto
        ind = np.random.choice(range(self.zdiscr[0]**2), p=pf_weighted) #Get the new (enumerated) state index using the calculated dynamics
        newState = self.stateVect[ind] #Get the new state from the state vector
        return(newState)
    

    def export_metrics(self, x, y, time):
        np.save("klc_planning_"+ str(self.mode), np.array([x, y, time]))

