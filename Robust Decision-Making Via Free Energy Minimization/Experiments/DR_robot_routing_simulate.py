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
import requests
from io import BytesIO

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from sklearn.gaussian_process import GaussianProcessRegressor 
from multiprocessing import Pool

control_space_size = 5 # define action space size

U_space_1 = np.array(np.linspace((-0.5),(0.5),control_space_size)) # define action space for 1st input
U_space_2 = np.array(np.linspace((-0.5),(0.5),control_space_size)) # define action sppace for 2nd input


time_step = 0.033

# # URL to the raw .dump file on GitHub
# url = 'https://github.com/jamalihuzaifa9/Thermal-Modelling-and-Parameter-Estimation-of-Power-Transformer/raw/refs/heads/master/GP_nominal.dump'

# # Download the file
# response = requests.get(url)

# # Check if the request was successful
# if response.status_code == 200:
#     # Load the pickle file from the response content (as a binary stream)
#     with BytesIO(response.content) as file:
#         GP_nominal = pickle.load(file)
#     print("Model loaded successfully!")
# else:
#     print(f"Failed to download the file. Status code: {response.status_code}")

GP_nominal= pickle.load(open(r'D:\Network Security\KL Control\robotarium_python_simulator\rps\examples\DR_FREE\Experiments\GP_nominal_1.dump','rb'))


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
        gauss_sum += 20*logpdf(state[:2],obs_points[:2,i],covar)
        
    cost = 50*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) + gauss_sum + 2*(np.exp(-0.5*((state[0]-(-1.5))/0.03)**2)/(0.03*np.sqrt(2*np.pi)) 
                + np.exp(-0.5*((state[0]-1.5)/0.03)**2)/(0.03*np.sqrt(2*np.pi)) + np.exp(-0.5*((state[1]-1.0)/0.03)**2)/(0.03*np.sqrt(2*np.pi)) 
                + np.exp(-0.5*((state[1]-(-1.0))/0.03)**2)/(0.03*np.sqrt(2*np.pi)))
    
    return(cost)

def state_cost_with_weights(state,goal_points,obs_points,weights):
    """
    Calculate state cost considering goal and obstacles using weights
    Args:
        state (array): Current state
        goal_points (array): Goal points
        obs_points (array): Obstacle points
        weights (array): Weights for cost computation
    Returns:
        float: State cost
    """
    
    v = np.array([0.025, 0.025], dtype=np.float32)
    covar = np.diag(v)
    
    gauss_sum = 0
    
    for i in range(np.size(obs_points,axis=1)):
        gauss_sum += -weights[0,i+1]*logpdf(state[:2],obs_points[:2,i],covar)
        
    cost = -weights[0,0]*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) + gauss_sum + 5*(np.exp(-0.5*((state[0]-(-1.5))/0.03)**2)/(0.03*np.sqrt(2*np.pi)) 
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


def C_tilde(Costs,eta,nominal_prob,reference_prob):
    """
    Calculate C_tilde for cost transformation
    Args:
        Costs (array): Costs array
        eta (float): radius of the KL ball
    Returns:
        float: Transformed cost
    """
    
    def objective(alpha):
        alpha = alpha[0]
        if alpha < 0.0:
            return np.inf  # Return a large number if alpha is non-positive
        term1 = alpha * eta

        term2 = alpha * np.log(np.sum(nominal_prob*((nominal_prob/reference_prob)*np.exp(np.array(Costs)))**(1/alpha)))
        return term1 + term2
    
    # Initial guess for alpha
    initial_guess = [1.0]

    # Constraint definition for scipy.optimize.minimize
    constraints = [
        {'type': 'ineq', 'fun': lambda alpha: alpha[0]-0.0}  # alpha > 0
    ]
    
    # Solve the problem
    result = minimize(objective, initial_guess,constraints=constraints)
    c_tilde = np.min([np.max(np.log((nominal_prob/reference_prob)*np.exp(np.array(Costs)))),result.fun])
    
    return c_tilde

    
def Control_step(state,U_space_1,U_space_2,goal_points,obs_points):
    
    """
    Perform an control step given an expert input
    Args:
        state (array): Current state
        U_space_1 (array): Action space for 1st input
        U_space_2 (array): Action space for 2nd input
        goal_points (array): Goal points
        obs_points (array): Obstacle points
    Returns:
        array: Action
    """
    exponent = np.zeros((control_space_size,control_space_size)) #Initialize pf
    pf = np.zeros((control_space_size,control_space_size)) #Initialize pf
    for i in range(control_space_size):
        for j in range(control_space_size):
            test_input = np.hstack((state.reshape(-1,), np.array([U_space_1[i],U_space_2[j]]))).reshape(1, -1)
            # next_state_nominal = nominal_model_step(state,np.array([U_space_1[i],U_space_2[j]]),time_step)
            next_state_nominal, sigma_nom = GP_nominal.predict(test_input,return_cov=True)
            cov_nom = np.diag(sigma_nom.reshape((2,)))
            # cov_nom = np.array([[0.0001, 0.00002], [0.00002, 0.0001]])
            p_bar = st.multivariate_normal(next_state_nominal.reshape((2,)),cov_nom)
            N_samples = 10
            next_sample = p_bar.rvs(N_samples)
            nominal_pdf = p_bar.pdf(next_sample)
            nominal_prob = nominal_pdf/np.sum(nominal_pdf) 
            
            
            next_state_reference = goal_points[:-1]
            cov_reference = np.array([[0.0001, 0.00002], [0.00002, 0.0001]])
            q = st.multivariate_normal(next_state_reference.reshape((2,)),cov_reference)
            nextq_sample= q.rvs(N_samples)
            reference_pdf = q.pdf(nextq_sample)
            reference_prob = reference_pdf/np.sum(reference_pdf)
           
            
            
            eta = np.clip(calculate_kl_divergence(goal_points[:-1].reshape((2,)),cov_reference,next_state_nominal.T.reshape((2,)),cov_nom),0.0,100.0)
            
            DKL = calculate_kl_divergence(next_state_nominal.T.reshape((2,)),cov_nom,goal_points[:-1].reshape((2,)),cov_reference)
            
            cost = [state_cost(next_sample[i,:],goal_points,obs_points) for i in range(N_samples)]
            
            # DR algorithm ##########################################
            c_t = C_tilde(cost,eta,nominal_prob,reference_prob)
            exponent[i,j] = -eta-c_t
            ###########################################################
            
            # uncomment following line to implement FPD, and comment line no. 318 and 319
            # exponent[i,j] = -DKL-np.sum(cost)/N_samples

    
    exp_max = np.max(exponent)
    pf = np.exp(exponent-exp_max)
    S2 = np.sum(pf) #Normalize resulting policy
    S2 = np.sum(pf) #Normalize resulting policy

    pf = np.array([x/S2 for x in pf])
            
    flat = pf.flatten()

    sample_index = np.random.choice(a=flat.size, p=flat)

    # Take this index and adjust it so it matches the original array
    adjusted_index = np.unravel_index(sample_index, pf.shape)

    action = np.reshape(np.array([U_space_1[adjusted_index[0]],U_space_2[adjusted_index[1]]]),(2,1))
    
    return(action,pf)
    

# Define goal points and obstacle points by removing orientation from poses
goal_points = np.array(np.mat('-1.4; -0.8; 0')) # you can change the goal points here
# obs_points_f = np.array(np.mat('0 0 0 0 0 0.8 0.8 0.8 0.8 0.8 -0.8 -0.8 -0.8 -0.8 -0.8;-0.8 -0.4 0 0.4 0.8 -0.8 -0.4 0 0.4 0.8 -0.8 -0.4 0 0.4 0.8;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0'))

# obstacle points defined here the the first elements before first ';' are the x axis coordinates and 2 set of elements are y axis co-ordinates (3rd set it is for pose we leave it to zero)
obs_points = np.array(np.mat('0 0 0 0 0 -0.8;0 0.2 0.4 0.6 0.8 -0.8;0 0 0 0 0 0'))

# Instantiate Robotarium object
N = 1
M = 4

# initial_conditions = [np.array(np.mat('1.3;0.9; 0')),np.array(np.mat('0.2;0.9; 0')),np.array(np.mat('1.3;-0.5; 0')),np.array(np.mat('-1.0;0.8; 0'))]
initial_conditions = [np.array(np.mat('1.32;0.9; 0')),np.array(np.mat('0.5;-0.2; 0')),np.array(np.mat('1.2;-0.5; 0')),np.array(np.mat('-0.5;0.25; 0'))] # can change robot initial condition in this line
# initial_conditions = [np.array(np.mat('1.0;0.8; 0')),np.array(np.mat('0.5;-0.2; 0')),np.array(np.mat('-0.5;-0.5; 0')),np.array(np.mat('-0.9;0.25; 0'))] # can change robot initial condition in this line
# initial_conditions = [np.array(np.mat('-1.1;0.9; 0')),np.array(np.mat('0.9;-0.2; 0')),np.array(np.mat('-0.7;0.5; 0')),np.array(np.mat('-0.5;0.25; 0'))]

XX = [0]*M
UU = [0]*M
XN = [0]*M
COVN = [0]*M

# Run Simulation

for I in range(M):
    
    D_xi = []
    X_si = []
    X_si_nom = []
    Cov_si_nom = []

    r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions[I], sim_in_real_time=False)

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

    # Create Goal Point Markers
    #Text with goal identification
    goal_caption = ['G{0}'.format(ii) for ii in range(goal_points.shape[1])]
    #Plot text for caption
    goal_points_text = [r.axes.text(goal_points[0,ii], goal_points[1,ii], goal_caption[ii], fontsize=font_size, color='k',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=-2)
    for ii in range(goal_points.shape[1])]
    goal_markers = [r.axes.scatter(goal_points[0,ii], goal_points[1,ii], s=marker_size_goal, marker='s', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width,zorder=-2)
    for ii in range(goal_points.shape[1])]
    

    # Create obstacle markers
    #Text with obstacle identification
    obs_caption = ['OBS{0}'.format(ii) for ii in range(obs_points.shape[1])]
    #Plot text for caption
    obs_points_text = [r.axes.text(obs_points[0,ii], obs_points[1,ii], obs_caption[ii], fontsize=font_size, color='k',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=-2)
    for ii in range(obs_points.shape[1])]
    obs_markers = [r.axes.scatter(obs_points[0,ii], obs_points[1,ii], s=marker_size_obs, marker='s', facecolors='none',edgecolors=CM[ii+1,:],linewidth=line_width,zorder=-2)
    for ii in range(obs_points.shape[1])]
    
    # gt_img = plt.imread(r'robotarium_environemnt.jpg')
    # x_img = np.linspace(-1.5, 1.5, gt_img.shape[1])
    # y_img = np.linspace(-1.0, 1.0, gt_img.shape[0])

    # gt_img_handle = r.axes.imshow(gt_img, extent=(-1.7, 1.67, -1.091, 1.07))
    
    position_history=np.empty((2,0))
    k = 0
    r.step()
    
        # While the number of robots at the required poses is less
    while (np.size(at_pose(np.vstack((x_si,x[2,:])), goal_points, position_error=0.25,rotation_error=100)) != N):

        # Get poses of agents
        x = r.get_poses()
        x_si = uni_to_si_states(x)
        X_si.append(x_si)
        
        obs_points_col = obs_points[:-1]
        # Loop through each column in obs_points and compare with X
        for i in range(obs_points.shape[1]):
            column = obs_points[:, i]
    
            # Check if the current column matches X
            if np.array_equal(column, x_si):
                print(f"Crashed into obstacle at {i}: {column}")
                break
            
        if x_si[0] <= np.array([-1.5]) or x_si[0] >= np.array([1.5])  or x_si[1] <= np.array([-1.0]) or x_si[1] >= np.array([1.0]):
            print('Touched the boundary wall')
            
            break
        
        if is_inside_rectangle(x_si, [-0.25,-0.25],[0.15,0.85] ):
            print(f"State entered the rectangle at: {x_si}")
            break
        
        if is_inside_rectangle(x_si, [-1.25,-1.0],[-0.51,-0.6]):
            print(f"State entered the rectangle at: {x_si}")
            break
        
        position_history=np.append(position_history, x[:2],axis=1)
        r.axes.scatter(position_history[0,:],position_history[1,:], s=1, linewidth=4, color='b',linestyle='dashed')

        for j in range(goal_points.shape[1]):
            goal_markers[j].set_sizes([determine_marker_size(r, goal_marker_size_m)])
        
        for j in range(obs_points.shape[1]):
            obs_markers[j].set_sizes([determine_marker_size(r, obs_marker_size_m)])

        # Create single-integrator control inputs
        
        dxi, u_pf = Control_step(x_si,U_space_1,U_space_2,goal_points,obs_points)
        D_xi.append(dxi)
        

        test_input = np.hstack((x_si.reshape(-1,), dxi.reshape(-1,))).reshape(1, -1)
        x_nom, sigma_nom = GP_nominal.predict(test_input,return_std=True)
        X_si_nom.append(x_nom)
        Cov_si_nom.append(sigma_nom)
        # Transform single integrator velocity commands to unicycle
        dxu = si_to_uni_dyn(dxi, x)
        
        k += 1
        if k==3000:
            break

        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        r.set_velocities(np.arange(N), dxu)
        # Iterate the simulation
        r.step()
    
    # save the state and control trajectory data  
    UU[I] = D_xi
    XX[I] = X_si
    XN[I] = X_si_nom
    COVN[I] = Cov_si_nom

    #Call at end of script to print debug information and for your script to run on the Robotarium server properly
    r.call_at_scripts_end()
    
XX = np.array(XX,dtype=object)
UU = np.array(UU,dtype=object)
XN = np.array(XN,dtype=object)
COVN = np.array(COVN,dtype=object)

np.save(r'State_Data_Simulation_DR.npy',XX)
np.save(r'Input_Data_Simulation_DR.npy',UU)
np.save(r'State_Data_nom.npy',XN)
np.save(r'COV_Data_nom.npy',COVN)
