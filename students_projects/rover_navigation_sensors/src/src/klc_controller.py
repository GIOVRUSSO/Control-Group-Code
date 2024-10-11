import pathlib
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import norm
from math import log
from obstacle_manager import *
import rospy
from Plants.uniform_plant import *
from Plants.linearized_plant import *
from Plants.trajectory_based_plant import *
import time
from pylab import rcParams
from utility import *
from cost_cache import *

from  obstacle_cluster_manager import ObstacleClusterManager
from state_manager_node import StateManager


"""
ControllerKLC: A class implementing a Kullback-Leibler Controller (KLC) for robot motion planning.

This class defines methods to initialize the controller and update its behavior based on robot states.
"""
class ControllerKLC:
    
    """
    This method nitialize the ControllerKLC class. 
    It initializes the state manager node, obstacle cluster manager, cost cache, and set the mode.
    It defines the distance threshold for the target.
    It sets the target anda the parameters for the discretization of the state space.
    It initializes the passive dynamics the probability matrix P and the state vector.
    It computes and export the cost plots.
    
    :param goal: The goal to set.
    :param mode: The mode to set.
    :param distance_threshold: The distance threshold for the target.
    
    :default distance_threshold: 0.5
    """
    def __init__(self, goal, mode, distance_threshold=0.5):
        

        
        self.state_manager_node = StateManager()
        self.obstacle_cluster_manager = ObstacleClusterManager()

        self.cache = CostCache()
        self.mode = mode
        
        self.distance_threshold = distance_threshold
        

        rospy.init_node('husky', anonymous=True)
        # Get the trasformation between odom and map
        self.init_position = get_position()
    
        self.cache.set_T(self.init_position)

        self.obstacles = ObstacleManager()
        
        self.poses = [] 

        #Target definition
        self.goal = goal
        self.xd = goal[0]
        self.yd = goal[1]
        
        #Dimensions of the variables
        self.zdim = 2

        #Minimum values
        self.zmin = [0, 0] 

        #Discretization steps
        self.zstep = [0.3, 0.3]

        #Amount of discrete bins
        self.zdiscr = [30, 30]

        #Number of iterations for the simulations
        #self.zsim = 15
        
        #Duration of the simulation
        self.duration = 30
    
        self.cost = np.zeros((self.zdiscr[0],self.zdiscr[1]))

        # Creazione del vettore 4D inizializzato con zeri

        self.passive_dynamics = np.zeros((self.zdiscr[0], self.zdiscr[0], self.zdiscr[0], self.zdiscr[0]))
        
        if mode == 0:
            self.passive_dynamics = uniform_plant().get_plant(self.zdiscr[0])
        elif mode == 1:
            self.passive_dynamics = linearized_plant().get_plant(2)
        elif mode == 2:
            self.passive_dynamics = trajectory_based_plant().get_plant(2, uniform_plant().get_plant(self.zdiscr[0]))   


        self.stateVect = np.zeros((self.zdiscr[0]**2, 2)) # Enumeration of all the possible states of the system
        self.discStates = np.zeros((self.zdiscr[0]**2, 2))

        for i in range(self.zdiscr[0]):
            #Enumerate the states from 1 to 30^2. Here we explicitly build the enumeration to later build the Q matrix and for convenience
            for j in range(self.zdiscr[0]):
                # Compute the angle and speed values for the current state
                x = self.zmin[0]+(i)*self.zstep[0]
                y = self.zmin[1]+(j)*self.zstep[0]
                
                # Calculate the index of the current state in the state vector
                ind = i*self.zdiscr[0] + j #Linearized index
                
                # Assign the angle and speed values to the state vector
                self.stateVect[ind] = [x, y] 
                self.discStates[ind]=[i,j]

        self.Prob = np.zeros((self.zdiscr[0]**2, self.zdiscr[0]**2)) #Initialize the probability matrix P

        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[0]):
                pf = self.passive_dynamics[i,j] #Get the pf corresponding to the passive dynamics
                ind1 = i*self.zdiscr[0] + j
                self.Prob[ind1] = self.unravelPF(pf) #Unravel and assign the pf to the probability matrix

        self.z = np.array((np.shape(self.Prob))[0])
        
        self.export_cost_plots(self.dynamic_cost)
        
    """
    Set the goal for the controller.
        
    :param goal: The goal to set.
    """
    def set_goal(self, goal):

        self.goal = goal
        self.xd = goal[0]
        self.yd = goal[1]

    """
    Update the controller's behavior based on the current state.
    
    :param self: The instance of the class.
    
    :return: Lists containing mean x position, mean y position, time, stdv x position, stdv y position.
    """   
    def update(self):


        state = self.state_manager_node.get_2D_position()
        print("State: " + str(state))
        nSteps = self.duration
        self.poses.append(state)
        np.save('poses.npy',self.poses)

        diagMinusQ = np.zeros((self.zdiscr[0]**2, self.zdiscr[0]**2)) #Q matrix
        hist = [[0,0]]*nSteps
        
        for i in range(self.zdiscr[0]):
            for j in range(self.zdiscr[1]):
                k = i*self.zdiscr[0]+j
                
                diagMinusQ[k,k] = np.exp(-self.dynamic_cost(self.stateVect[k])) #Cost function

        self.z = self.power_method(diagMinusQ @ self.Prob, self.zdiscr[0]**2)
                
        for t in range(nSteps):

            ind = int(state[0])*self.zdiscr[0]+int(state[1]) #Get the index of the current state
            hist[t]=self.stateVect[ind] #Log the state
            state = self.loop(state) #Sample the new state
            t+=1
            
            distance_to_target = np.linalg.norm(np.array(state) - np.array(self.goal))

            #Stopping condition: if the distance is less than a defined threshold
            if distance_to_target < self.distance_threshold:
                #print(f"Arrivato al target in {t} passi.")
                nSteps = t
                break
            
        waypoints_x = [pt[0] for pt in hist]
        waypoints_y = [pt[1] for pt in hist]
        # print("Waypoints x: ", waypoints_x)
        # print("Waypoints y: ", waypoints_y)
        
        meanx = [0]*nSteps #Get the means and stds for plotting
        stdsx = [0]*nSteps
        for i in range(nSteps):
            meanx[i] = np.mean(waypoints_x)
            stdsx[i] = np.std(waypoints_x)

        meany = [0]*nSteps #Get the means and stds for plotting
        stdsy = [0]*nSteps
        for i in range(nSteps):
            meany[i] = np.mean(waypoints_y)
            stdsy[i] = np.std(waypoints_y)

        htime = np.array([time for time in range(self.duration)])

        self.cache.set_targets(waypoints_x, waypoints_y)
        return [meanx, meany, htime, stdsx, stdsy]

    
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
    This method calculates the cost of a given state.
    It calculates the cost based on the distance from the goal, the obstacles, and the obstacle clusters and the position of the goal.
    
    :param state: The state to calculate the cost for.
    :param debug: The debug flag.
    
    :return: The calculated cost.
    
    """
    def dynamic_cost(self, state, debug=False):

        """
        This method calculates the value of the probability density function of a multivariate normal distribution given the mean and the covariance matrix.
        
        :param x: The input vector.
        :param u: The mean vector.
        :param covar: The covariance matrix.
        """ 
        def my_logpdf(x, u, covar):

            k = len(x)  # dimension
            a = np.transpose(x - u)
            b = np.linalg.inv(covar)
            c = x - u
            d = np.matmul(a, b)
            e = np.matmul(d, c)
            numer = np.exp(-0.5 * e)
            f = (2 * np.pi)**k
            g = np.linalg.det(covar)
            denom = np.sqrt(f * g)
            pdf = numer / denom
            return pdf
        
        obs_points = self.obstacles.get_obs()
        #rospy.loginfo("Waiting for obstacle update...")
        while not self.obstacles.check_update():
            
            obs_points = self.obstacles.get_obs()
        #rospy.loginfo("Got obstacles.")
        v = np.array([0.2, 0.2], dtype=np.float32)
        covar = np.diag(v)
        gauss_sum = 0
        
        obs_points_c = np.array([])
        
        for obs in obs_points:
            obs_point  = np.array([obs[0], obs[1]])
            
            gauss_sum += 30*my_logpdf(state[:2],obs_point,covar)
            np.append(obs_points_c, obs_point)

        self.obstacle_cluster_manager.set_obstacle_points(obs_points_c)
        self.obstacle_cluster_manager.obstacle_clustering()
        obstacle_centroids = self.obstacle_cluster_manager.get_obstacle_clusters_centroids()
        
        covar_cluster= np.diag(np.array([0.5, 0.5], dtype=np.float32))
        cluster_sum = 0
        for i in range(len(obstacle_centroids)): 
            dist= np.sqrt((self.xd-obstacle_centroids[i][0])**2+(self.yd-obstacle_centroids[i][1])**2) 
            dist=dist if dist < 0.65 else 1 
            cluster_sum += 50*my_logpdf(state[:2],obstacle_centroids[i][:2],covar_cluster)*(obstacle_centroids[i][2]-1)*dist 
        
        distance_from_goal = np.linalg.norm(np.array(state) - np.array(self.goal))
        
        target_covariance = np.array([0.1, 0.1], dtype=np.float32)
        covar_target = np.diag(target_covariance)
        
        # This term is used to get a lower cost when the state is closer to the goal
        target_cost = -7 * my_logpdf(state[:2], self.goal[:2], covar_target)
        
        cost = 5.25 * distance_from_goal + gauss_sum + cluster_sum + target_cost
        
        return cost
    
    """
    This method exports the cost plots.
    It plots cost function and its gradient in 3D.
    
    :param cost_function: The cost function to use.
    :param directory_path: The directory path to save the plots.
        
    """   
    
    def export_cost_plots(self,cost_function, directory_path = pathlib.Path(__file__).parent):

        
        X, Y = np.meshgrid(np.linspace(-10, 10, 100), np.linspace(-10, 10, 100))
        z = np.array([cost_function(np.array([x,y])) for x,y in zip(np.ravel(X), np.ravel(Y))])
        Z = z.reshape(X.shape)
        grad = np.gradient(Z)
        fig_3d = plt.figure()
        ax = fig_3d.add_subplot(projection='3d')
        ax.plot_surface(X, Y, Z)
        fig_3d.savefig(directory_path / "cost_3d.png")
        fig_grad = plt.figure()
        ax = fig_grad.add_subplot()
        ax.quiver(X, Y, -grad[1], -grad[0], angles='xy')
        fig_grad.savefig(directory_path / "cost_3d_grad.png")
        return fig_3d, fig_grad
    
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
    def power_method(self, mat, dim, epsilon=1e-6):
        vect = np.ones(dim)
        nrm = np.linalg.norm(vect)
        
        for i in range(30):
            prev_vect = vect.copy() 
            vect = mat.dot(vect)
            nrm = np.linalg.norm(vect)
            # if nrm == 0:
            #     nrm = 0.0001
            vect = vect / nrm
            
            # Compute the difference between the current and the previous vector
            diff = np.linalg.norm(vect - prev_vect)
            
            #Verifica la condizione di arresto
            if diff < epsilon:
                break
        return vect

    
    """
    This method performs the control loop.
    
    :param x: The current state.
    :return: The new state.
    """
    def loop(self, x):
        #Control loop 
        
        ind = (int(x[0]),int(x[1]))
        
        #Get the pf corresponding to the passive dynamics
        pf = self.passive_dynamics[ind[0],ind[1]] #Get the pf corresponding to the passive dynamics
        pf_1D = self.unravelPF(pf) #Unravel it
        
        #Calculate the actual transition pf using z and the passive dynamics
        pf_weighted = np.multiply(pf_1D,self.z)
        
        #Normalize
        S = np.sum(pf_weighted) 

        pf_weighted = pf_weighted/S #probabilities contain NaN: it happens when S=0, i.e. when the passive dynamics are zero.
        
        #Sample new state
        ind = np.random.choice(range(self.zdiscr[0]**2), p=pf_weighted) #Get the new (enumerated) state index using the calculated dynamics
        newState = self.discStates[ind]#self.stateVect[ind] #Get the new state from the state vector
        
        return(newState)
    
    """
    This method checks if an obstacle is in the field of view of the rover.
    
    :param rover_x: The x coordinate of the rover.
    :param rover_y: The y coordinate of the rover.
    :param obs_x: The x coordinate of the obstacle.
    :param obs_y: The y coordinate of the obstacle.
    :param obs_radius: The radius of the obstacle.
    """
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
    This method exports the metrics.
    
    :param x: The x values to export.
    :param y: The y values to export.
    :param time: The time values to export.
    """
    def export_metrics(self, x, y, time):
        np.save("klc_online_real_planning_" + str(self.mode), np.array([x, y, time]))

    """
    This method computes the state vector.
    """
    def compute_state_vect(self):
        x_min = self.zmin[0]
        y_min = self.zmin[1]
        for i in range(self.zdiscr[0]):
            #Enumerate the states from 1 to 30^2. Here we explicitly build the enumeration to later build the Q matrix and for convenience
            for j in range(self.zdiscr[0]):
                # Compute the angle and speed values for the current state
                x = x_min + (i)*self.zstep[0]
                y = y_min + (j)*self.zstep[0]
                
                # Calculate the index of the current state in the state vector
                ind = i*self.zdiscr[0] + j #Linearized index
                
                # Assign the angle and speed values to the state vector
                self.stateVect[ind] = [x, y] 
                self.discStates[ind]=[i,j]
