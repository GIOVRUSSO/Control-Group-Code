import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
from time import time

import numpy as np
import scipy.stats as st
from scipy.optimize import minimize
import time
import pickle

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from sklearn.gaussian_process import GaussianProcessRegressor 

control_space_size = 3 # define action space size

U_space_1 = np.array(np.linspace((-1.0),(1.0),control_space_size)) # define action space for 1st input
U_space_2 = np.array(np.linspace((-1.),(1.),control_space_size)) # define action sppace for 2nd input

time_step = 0.033


WIND_DIRECTION = np.array([1, 1])  # Wind blowing in both positive x and y directions
WIND_SPEED = 0.8  # Wind speed
DRAG_COEFFICIENT = 1.0  # drag coefficient
AIR_DENSITY = 1.25 # Air density 
ROBOT_AREA = 1.0  # Approximate frontal area of the robot in m^2

def compute_wind_force():
    wind_force_magnitude = 1.0 * AIR_DENSITY * DRAG_COEFFICIENT * ROBOT_AREA * WIND_SPEED**2
    wind_force = wind_force_magnitude * WIND_DIRECTION / np.linalg.norm(WIND_DIRECTION)
    return wind_force


def model_step(x,velocities,time_step):
    """
    Actual System Model P(.)
    Args:
        x (array): State
        velocities (array): Control velocities
        time_step (float): Time step
    Returns:
        array: Next state
    """
    poses = np.zeros((2,1))
    
    # Update dynamics of agents
    poses[0] = x[0] + time_step*velocities[0]
    poses[1] = x[1] + time_step*velocities[1]
    
    return(poses)

def reference_model_step(x,velocities,time_step):
    """
    reference System Model Q(.)
    Args:
        x (array): State
        velocities (array): Control velocities
        time_step (float): Time step
    Returns:
        array: Next state
    """
    poses = np.zeros((2,1))
    
    # Update dynamics of agents
    poses[0] = 0.6*x[0] + time_step*velocities[0]
    poses[1] = 0.6*x[1] + time_step*velocities[1]
    
    return(poses)

def nominal_model_step(x,velocities,time_step):
    """
    Nominal System Model P^{bar}(.)
    Args:
        x (array): State
        velocities (array): Control velocities
        time_step (float): Time step
    Returns:
        array: Nominal next state
    """
    poses = np.zeros((2,1))
    
    # Update dynamics of agents
    poses[0] = 1.0*x[0] + time_step*velocities[0] + 0.1*x[0]
    poses[1] = 1.0*x[1] + time_step*velocities[1] + 0.1*x[1]
    
    return(poses)

def worst_model_step(x,velocities,time_step):
    """
    Worst System Model
    Args:
        x (array): State
        velocities (array): Control velocities
        time_step (float): Time step
    Returns:
        array: Worst case next state
    """
    poses = np.zeros((2,1))
    
    # Update dynamics of agents
    poses[0] = 1.5*x[0] + time_step*velocities[0]
    poses[1] = 1.5*x[1] + time_step*velocities[1]
    
    return(poses)

def logpdf(x, u, covar):
    """
    Gaussian kernel
    Args:
        x (array): Current state
        u (array): Obstacle points
        covar (array): Covariance
    Returns:
        float: Probability of hitting the obstacle
    """
    
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

import numpy as np


def goal_cost(state,goal_points):
    """
    Calculate cost based on the distance between goal point and current robot state
    Args:
        state (array): Current state
        goal_points (array): Goal points
    Returns:
        float: Cost
    """
        
    cost = 30*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) 
   
    return(cost)

def reference_input(state,U_space_1,U_space_2,goal_points):
    
    """
    Calculate reference input distribution q_u(.) for reaching the goal point
    Args:
        state (array): Current state
        U_space_1 (array): Action space for 1st input
        U_space_2 (array): Action space for 2nd input
        goal_points (array): Goal points
    Returns:
        array: Reference input distribution
    """
    time_step = 0.033
    
    qpf = np.zeros((control_space_size,control_space_size)) #Initialize q_u
    for i in range(control_space_size): # for all possible inputs in 1st axis
        for j in range(control_space_size): # for all possible inputs in 2nd axis
            next_state_reference = reference_model_step(state,[U_space_1[i],U_space_2[j]],time_step) # predicting next nominal state
            cov_nominal = np.array([[0.001, 0.0002], [0.0002, 0.001]])
            f = st.multivariate_normal(next_state_reference.reshape((2,)),cov_nominal) # creating nominal state distribution
            N_samples = 20
            next_sample = f.rvs(N_samples)
            
            cost = [goal_cost(next_sample[i,:],goal_points) for i in range(N_samples)] # calculating goal cost for N samples from nominal state distribution
            policy = np.exp(-np.sum(cost)/N_samples) 
            
            qpf[i,j] = policy
    S2 = np.sum(qpf) 
    
    qpf = np.array([x/S2 for x in qpf]) #Normalize resulting policy
            
    return(qpf)

def state_cost(state,goal_points,obs_points):
    """
    Calculate state cost considering goal and obstacles
    Args:
        state (array): Current state
        goal_points (array): Goal points
        obs_points (array): Obstacle points
    Returns:
        float: State cost
    """
    
    v = np.array([0.025, 0.025], dtype=np.float32)
    covar = np.diag(v)
    
    gauss_sum = 0
    
    for i in range(np.size(obs_points,axis=1)):
        gauss_sum += 10*logpdf(state[:2],obs_points[:2,i],covar)
        
    cost = 20*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) + gauss_sum + 5*(np.exp(-0.5*((state[0]-(-1.5))/0.03)**2)/(0.03*np.sqrt(2*np.pi)) 
                + np.exp(-0.5*((state[0]-1.5)/0.03)**2)/(0.03*np.sqrt(2*np.pi)) + np.exp(-0.5*((state[1]-1.0)/0.03)**2)/(0.03*np.sqrt(2*np.pi)) 
                + np.exp(-0.5*((state[1]-(-1.0))/0.03)**2)/(0.03*np.sqrt(2*np.pi)))
    
    return(cost)


def calculate_kl_divergence(mu1, cov1, mu2, cov2):
    """
    Calculate the Kullback-Leibler (KL) divergence between two multivariate Gaussian distributions.

    Args:
        mu1: numpy array of shape (d,), the mean vector of the first Gaussian distribution
        cov1: numpy array of shape (d, d), the covariance matrix of the first Gaussian distribution
        mu2: numpy array of shape (d,), the mean vector of the second Gaussian distribution
        cov2: numpy array of shape (d, d), the covariance matrix of the second Gaussian distribution

    Returns:
        kl_divergence: float, the KL divergence between the two Gaussian distributions
    """
    d = len(mu1)

    # Invert the covariance matrix of the second Gaussian distribution
    cov2_inv = np.linalg.inv(cov2)

    # Calculate the trace term
    trace_term = np.trace(np.matmul(cov2_inv, cov1))

    # Calculate the squared difference in means
    mean_diff = mu2 - mu1
    mean_diff_term = np.dot(np.dot(mean_diff, cov2_inv), mean_diff)

    # Calculate the log-determinant term
    log_det_term = np.log(np.linalg.det(cov2) / np.linalg.det(cov1))

    # Calculate the KL divergence
    kl_divergence = 0.5 * (trace_term + mean_diff_term - d + log_det_term)

    return kl_divergence


def is_inside_rectangle(state, bottom_left, top_right):
    """
    Check if the state is inside the rectangular region.

    Args:
        state (list or numpy.ndarray): State as a 2D point [x, y].
        bottom_left (list): Bottom-left corner of the rectangle [x_min, y_min].
        top_right (list): Top-right corner of the rectangle [x_max, y_max].

    Returns:
        bool: True if the state is inside the rectangle, False otherwise.
    """
    return bottom_left[0] <= state[0] <= top_right[0] and bottom_left[1] <= state[1] <= top_right[1]


wall_points_x = np.array([-1.4,1.4])
wall_points_y = np.array([-1.0,1.0])

# Instantiate Robotarium object
N = 1
initial_conditions = [np.array(np.mat('0.15;0.09; 0')),np.array(np.mat('-0.5;-0.2; 0')),np.array(np.mat('0.02;-0.05; 0')),np.array(np.mat('-0.05;0.25; 0'))] # can change robot initial condition in this line


XX = [0]*3
UU = [0]*3
XN = [0]*3

# Run Simulation

for I in range(3):
    
    D_xi = []
    X_si = []
    X_si_nom = []

    r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions[I], sim_in_real_time=False)

    #obs_points = np.array(np.mat('0 0 0 0 0 0.9 -0.8 -0.8;0 0.2 0.4 0.6 0.8 -0.7 -0.9 -0.8;0 0 0 0 0 0 0 0'))

    # Create single integrator position controller
    single_integrator_position_controller = create_si_position_controller()

    # Create barrier certificates to avoid collision
    #si_barrier_cert = create_single_integrator_barrier_certificate()
    si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

    _, uni_to_si_states = create_si_to_uni_mapping()

    # Create mapping from single integrator velocity commands to unicycle velocity commands
    si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

    # define x initially
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    X_si_nom.append(x_si)
    
    # Plotting Parameters
    CM = np.random.rand(N+10,3) # Random Colors
    goal_marker_size_m = 0.1
    obs_marker_size_m = 0.1
    #robot_marker_size_m = 0.1
    marker_size_goal = determine_marker_size(r,goal_marker_size_m)
    marker_size_obs = determine_marker_size(r,obs_marker_size_m)
    #marker_size_robot = determine_marker_size(r, robot_marker_size_m)
    font_size = determine_font_size(r,0.1)
    line_width = 5
    
    position_history = np.empty((2,0)) # History to show what the robot does


    r.step()
    
        # While the number of robots at the required poses is less
    # than N...
    # while (np.size(at_pose(np.vstack((x_si,x[2,:])), goal_points, position_error=0.25,rotation_error=100)) != N):
    for k in range(500):

        # Get poses of agents
        x = r.get_poses()
        x_si = uni_to_si_states(x)
        #print(x_si.shape)
        
        
            
        if x_si[0] <= np.array([-1.5]) or x_si[0] >= np.array([1.5])  or x_si[1] <= np.array([-1.0]) or x_si[1] >= np.array([1.0]):
            print('Touched the boundary wall')
            
            break
        
        
        # Plotting the robot's true trajectory.
        # position_history=np.append(position_history, x[:2],axis=1)
        # r.axes.scatter(position_history[0,:],position_history[1,:], s=1, linewidth=2, color='b',linestyle='dashed')
       
        velocities_range = [-0.5,-0.25,0.0,0.25,0.5]
        
        dxi = np.reshape(np.random.choice(velocities_range, 2),(2,1)) 
        D_xi.append(dxi)
        
        x_nom = nominal_model_step(x_si + 0.1*x_si,dxi,time_step)
        X_si_nom.append(x_nom)
        # Transform single integrator velocity commands to unicycle
        dxu = si_to_uni_dyn(dxi, x)
        X_si.append(x_si+0.0*x_si)


        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        r.set_velocities(np.arange(N), dxu)
        # Iterate the simulation
        r.step()
    
    # save the state and control trajectory data  
    UU[I] = D_xi
    XX[I] = X_si
    XN[I] = X_si_nom

    #Call at end of script to print debug information and for your script to run on the Robotarium server properly
    # r.call_at_scripts_end()
    
XX = np.array(XX,dtype=object)
UU = np.array(UU,dtype=object)
XN = np.array(XN,dtype=object)


np.save('State_Dataset_new.npy',XX)
np.save('Input_Dataset_new.npy',UU)
np.save('State_Data_Nominal.npy',XN)