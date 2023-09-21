import numpy as np
import matplotlib.pyplot as plt
import rospy
from Plants.uniform_plant import *
from Plants.linearized_plant import *
from Plants.trajectory_based_plant import *
from time import *
from pylab import rcParams
from cost_cache import *
from utility import *

"""
ControllerKLC: A class implementing a Kinematic Linearization Controller (KLC) for robot motion planning.

This class defines methods to initialize the controller and update its behavior based on robot states.
"""
class ControllerKLCOnline:

    """
    Update the controller's behavior based on the current state.
    
    :param self: The instance of the class.
    :return: Lists containing mean x position, mean y position, and time.
    """
    def __init__(self, goal, mode):

        self.cache = CostCache()

        #Target definition
        self.goal = goal
        self.xd = goal[0]
        self.yd = goal[1]
        
        #Dimensions of the variables
        self.zdim = 2

        #Minimum values
        self.zmin = [0, 0] 

        #Discretization steps
        self.zstep = [0.5, 0.5]

        #Amount of discrete bins
        self.zdiscr = [18, 18]

        #Number of iterations for the simulations
        self.zsim = 15
        #Duration of the simulation
        self.duration = 30

        # Creazione del vettore 4D inizializzato con zeri

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

        self.Prob = np.zeros((self.zdiscr[0]**2, self.zdiscr[0]**2))

        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[0]):
                pf = self.passive_dynamics[i,j]
                ind1 = i*self.zdiscr[0] + j
                self.Prob[ind1] = self.unravelPF(pf)

        self.z = np.array((np.shape(self.Prob))[0])

        self.world_odom_obstacles = [[0.5, 3.5], [4, 4], [1.7, 3.8], [2.5, 4.6], [4.8, 4.8], [8, 3]]
    
    """
    Update the controller's behavior based on the current state.
    
    :param self: The instance of the class.
    :return: Lists containing mean x position, mean y position, and time.
    """
    def update(self):
        fullH = np.zeros((self.zsim,self.duration))
        fullHv = np.zeros((self.zsim,self.duration))
        nSteps = self.duration
        diagMinusQ = np.zeros((self.zdiscr[0]**2, self.zdiscr[0]**2)) # q
        actions = 0.5*np.array([(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)])
        possible_states = []

        #Task:  obtain simulations for different initial conditions (say, 5 different initial conditions). For each of these, run 50 simulations.

        for j in range(self.zsim): #Perform simulations
            print("simulazione numero: " + str(j))
            hist = [[0,0]]*nSteps
            state = [0, 0] #Initialize the pendulum <------------
            for i in range(nSteps): #For each step                
                #compute new possible states from statevect using the current state
                
                #Verificare prima quali stati sono disponibili a partire da state per andare in state + action nella 
                #matrice stateVect
                #compute di index of the next states
                for action in actions:
                    is_good_state = state + action
                    #print(is_good_state)
                    if 0 <= is_good_state[0] < self.zdiscr[0]*self.zstep[0] and 0 <= is_good_state[1] < self.zdiscr[1]*self.zstep[1]:
                        state_index = 0
                        for s in self.stateVect:
                            if s[0] == is_good_state[0] and s[1] == is_good_state[1]:
                                #print(s)
                                possible_states.append(state_index)
                                #print(possible_states)
                            state_index+=1
                
                for k in possible_states:
                    #print("Il valore del vett in pos " + str(k) + " è: " + str(self.stateVect[k]))
                    #Build the diagonal matrix with the exponential of the opposite of the cost
                    diagMinusQ[k,k] = np.exp(-self.cost(self.stateVect[k]))

                self.z = self.powerMethod(diagMinusQ@self.Prob, self.zdiscr[0]**2)
                
                possible_states = []

                hist[i]=state #Log the state
                state = self.loop(state) #Sample the new state


            fullH[j] = [x[0] for x in hist]
            fullHv[j] = [x[1] for x in hist]

        meanx = [0]*self.duration #Get the means and stds for plotting
        stdsx = [0]*self.duration
        for i in range(self.duration):
            meanx[i] = np.mean(fullH[:,i])
            stdsx[i] = np.std(fullH[:,i])

        plt.rcParams.update({'font.size': 18})

        #PLOT X -> Angle
        x = np.array([x for x in range(self.duration)])
        y = np.array(meanx)
        ci = np.array(stdsx)

        fig, ax = plt.subplots()
        plt.xlim([0, self.duration])
        ax.plot(x,y)
        plt.xlabel("Time")
        plt.ylabel("State")
        plt.title("Position on x")
        ax.fill_between(x, (y-ci), (y+ci), color='b', alpha=.1)
        plt.show()

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

        plt.rcParams.update({'font.size': 18})

        #PLOT X -> Angle
        x = np.array([x for x in range(self.duration)])
        y = np.array(meany)
        ci = np.array(stdsy)

        fig, ax = plt.subplots()
        plt.xlim([0, self.duration])
        ax.plot(x,y)
        plt.xlabel("Time")
        plt.ylabel("State")
        plt.title("Position on y")
        ax.fill_between(x, (y-ci), (y+ci), color='b', alpha=.1)
        plt.show()

        return [meanx, meany, time]

    
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

        obsTerm = 0

        #ADD VISION OF THE ROBOT FOR THE OBSTACLES
        for obs in self.obstacles.get_obs():
            if(self.is_obstacle_in_fov(state[0], state[1], obs[0], obs[1], 1) == True):
                xterm = ((state[0] - obs[0]) / sx) ** 2
                yterm = ((state[1] - obs[1]) / sy) ** 2
                obsTerm += k * np.exp(-0.5 * (xterm + yterm))

        return 0.7*(state[0] - self.xd) ** 2 + 0.7*(state[1] - self.yd) ** 2 + obsTerm 


    def is_obstacle_in_fov(self, rover_x, rover_y, obs_x, obs_y, obs_radius):
            
        fov_radius = 1.5  # Radius of the rover's field of view
            
        # Calculate the distance between the rover and the obstacle
        distance = np.sqrt((rover_x - obs_x)**2 + (rover_y - obs_y)**2)
        
        # Calculate the sum of the rover's radius and the obstacle's radius
        total_radius = fov_radius + obs_radius
        
        # Check if the obstacle is within the rover's field of view considering both radii
        if distance <= total_radius:
            return True  # Obstacle is in the field of view
        else:
            return False  # Obstacle is not in the field of view
    
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
            prev_vect = vect.copy()  # Salva l'autovettore dell'iterazione precedente
            vect = mat.dot(vect)
            nrm = np.linalg.norm(vect)
            vect = vect / nrm
            
            """# Calcola la differenza tra l'autovettore attuale e quello precedente
            diff = np.linalg.norm(vect - prev_vect)
            
            # Verifica la condizione di arresto
            if diff < epsilon:
                break"""
        
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
        np.save("klc_vision_linear_results_from_planning", np.array([x, y, time]))
