import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
from time import time

import numpy as np
import scipy.stats as st
import time

# weights = np.load(r'D:\Network Security\KL Control\robotarium_python_simulator\rps\examples\go_to_point\Weights.npy')
control_space_size = 3

U_space_1 = np.array(np.linspace((-0.15),(0.15),control_space_size))
U_space_2 = np.array(np.linspace((-0.15),(0.15),control_space_size))

time_step = 0.033

#weights = np.array([-3.00613544e+01,-1.35285679e+01,-3.16906745e+01,-3.42397218e+01,-1.82064449e+01,-9.78983289e-01,-3.14145396e-01,2.16345019e-01,-1.99320125e+01,3.33850141e-01,1.98431644e-02,-3.61292790e-01])
#weights = np.array([-13.82806378,-7.37552209,-19.5130216,-16.57098794,-6.44948079,-0.56244083, 0.08046996,-0.72077351,-12.43774902,-0.02072203,0.07882772,0.7128548])

def model_step(x,velocities,time_step):
    poses = np.zeros((2,1))
    
    # Update dynamics of agents
    poses[0] = x[0] + time_step*velocities[0]
    poses[1] = x[1] + time_step*velocities[1]
    
    return(poses)


def lb_ub_support(mu,cov):
    
    # Calculate the lower and upper bounds with 95% confidence
    z = st.norm.ppf(0.975) 
    lower_bound = mu - z * np.diag(np.linalg.cholesky(cov))
    upper_bound = mu + z * np.diag(np.linalg.cholesky(cov))
    
    return(lower_bound,upper_bound)

def q_constant(lower_bound,upper_bound):
    length = np.array(upper_bound-lower_bound)

    area = np.product(length)
    
    q_constant = 1/area
    
    return(q_constant)

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

import numpy as np

def multivariate_rbf_kernel(X1, X2, gamma):
    #Compute the multivariate Radial Basis Function (RBF) kernel between two sets of input points.
    n1 = X1.shape[0]
    n2 = X2.shape[0]
    K = np.zeros((n1, n2))

    diff = X1[:] - X2[:]
    K = np.exp(-gamma * np.dot(diff, diff))

    return K


# def state_cost_w(state,goal_points,obs_points,weights):
#     v = np.array([0.025, 0.025], dtype=np.float32)
#     covar = np.diag(v)
#     #cost = 60*(state[0]-goal_points[0])**2 + 60*(state[1]-goal_points[1])**2 + 200*(np.exp(-(np.abs(state[0]-obs_points[0,0])+np.abs(state[1]-obs_points[1,0]))) + np.exp(-(np.abs(state[0]-obs_points[0,1]) + np.abs(state[1]-obs_points[1,1]))) + np.exp(-(np.abs(state[0]-obs_points[0,2])+np.abs(state[1]-obs_points[1,2])))) #actual cost
    
#     #cost = 70*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) + 100*(my_logpdf(state[:2],obs_points[:2,0],covar) + my_logpdf(state[:2],obs_points[:2,1],covar) + my_logpdf(state[:2],obs_points[:2,2],covar) + my_logpdf(state[:2],obs_points[:2,3],covar) + my_logpdf(state[:2],np.array([-1.7,-1]),covar) + my_logpdf(state[:2],np.array([1.7,1]),covar))
    
#     gauss_sum = 0
    
#     for i in range(np.size(obs_points,axis=1)):
#         # gauss_sum += -weights[:,i+1]*my_logpdf(state[:2],obs_points[:2,i],covar)
#         gauss_sum += -weights[:,i+1]*multivariate_rbf_kernel(state[:2],obs_points[:2,i],20)
        
#     cost = -weights[:,0]*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) + gauss_sum + 10*(np.exp(-0.5*((state[0]-(-1.5))/0.03)**2)/(0.03*np.sqrt(2*np.pi)) 
#                 + np.exp(-0.5*((state[0]-1.5)/0.03)**2)/(0.03*np.sqrt(2*np.pi)) + np.exp(-0.5*((state[1]-1.0)/0.03)**2)/(0.03*np.sqrt(2*np.pi)) 
#                 + np.exp(-0.5*((state[1]-(-1.0))/0.03)**2)/(0.03*np.sqrt(2*np.pi)))
    
#     #cost = 30*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) + gauss_sum + 10*(np.exp(-0.5*((state[0]-(-1.5))/0.02)**2)/(0.02*np.sqrt(2*np.pi)) 
#     #            + np.exp(-0.5*((state[0]-1.5)/0.02)**2)/(0.02*np.sqrt(2*np.pi)) + np.exp(-0.5*((state[1]-1.0)/0.02)**2)/(0.02*np.sqrt(2*np.pi)) 
#     #            + np.exp(-0.5*((state[1]-(-1.0))/0.02)**2)/(0.02*np.sqrt(2*np.pi)))
    
#     return(cost)

def state_cost(state,goal_points,obs_points):
    v = np.array([0.025, 0.025], dtype=np.float32)
    covar = np.diag(v)
    #cost = 60*(state[0]-goal_points[0])**2 + 60*(state[1]-goal_points[1])**2 + 200*(np.exp(-(np.abs(state[0]-obs_points[0,0])+np.abs(state[1]-obs_points[1,0]))) + np.exp(-(np.abs(state[0]-obs_points[0,1]) + np.abs(state[1]-obs_points[1,1]))) + np.exp(-(np.abs(state[0]-obs_points[0,2])+np.abs(state[1]-obs_points[1,2])))) #actual cost
    
    #cost = 70*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) + 100*(my_logpdf(state[:2],obs_points[:2,0],covar) + my_logpdf(state[:2],obs_points[:2,1],covar) + my_logpdf(state[:2],obs_points[:2,2],covar) + my_logpdf(state[:2],obs_points[:2,3],covar) + my_logpdf(state[:2],np.array([-1.7,-1]),covar) + my_logpdf(state[:2],np.array([1.7,1]),covar))
    
    gauss_sum = 0
    
    for i in range(np.size(obs_points,axis=1)):
        gauss_sum += 20*my_logpdf(state[:2],obs_points[:2,i],covar)
        # gauss_sum += 20*multivariate_rbf_kernel(state[:2],obs_points[:2,i],20)
        
    cost = 30*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) + gauss_sum + 10*(np.exp(-0.5*((state[0]-(-1.5))/0.03)**2)/(0.03*np.sqrt(2*np.pi)) 
                + np.exp(-0.5*((state[0]-1.5)/0.03)**2)/(0.03*np.sqrt(2*np.pi)) + np.exp(-0.5*((state[1]-1.0)/0.03)**2)/(0.03*np.sqrt(2*np.pi)) 
                + np.exp(-0.5*((state[1]-(-1.0))/0.03)**2)/(0.03*np.sqrt(2*np.pi)))
    
    # cost = 30*((state[0]-goal_points[0])**2 + (state[1]-goal_points[1])**2) + gauss_sum + 10*(np.exp(-0.5*((state[0]-(-1.5))/0.02)**2)/(0.02*np.sqrt(2*np.pi)) 
    #            + np.exp(-0.5*((state[0]-1.5)/0.02)**2)/(0.02*np.sqrt(2*np.pi)) + np.exp(-0.5*((state[1]-1.0)/0.02)**2)/(0.02*np.sqrt(2*np.pi)) 
    #            + np.exp(-0.5*((state[1]-(-1.0))/0.02)**2)/(0.02*np.sqrt(2*np.pi)))
    return(cost)

def C_Bar(state,goal_points):
    
    #ind = discretize(state, 2, [-np.pi, -5], [2*np.pi/50, 0.2])
    Cost = np.zeros((10,10)) #initialize cost
    #Cost = 0
    
    for i in range(10):
            for j in range(10):
                next_state = model_step(state,[U_space_1[i],U_space_2[j]],time_step)
                cov = np.array([[1, 0.5, 0.3], [0.5, 1, 0.2], [0.3, 0.2, 1]])
                f = st.multivariate_normal(next_state.reshape((3,)),cov)
                next_state = f.rvs()
                cost = state_cost(next_state,goal_points)
                Cost[i,j] = cost
    
    Expected_Cost = np.sum(Cost*(1/100))
                
    return(Expected_Cost)
    
def Control_step(state,U_space_1,U_space_2,goal_points,obs_points):
        ###
        target_pf = 1
        time_step = 0.033
    
        
        pf = np.zeros((control_space_size,control_space_size)) #Initialize pf
        for i in range(control_space_size):
            for j in range(control_space_size):
                next_state = model_step(state,[U_space_1[i],U_space_2[j]],time_step)
                cov = np.array([[0.001, 0.0002], [0.0002, 0.001]])
                f = st.multivariate_normal(next_state.reshape((2,)),cov)
                
                N_samples = 5
                next_sample = f.rvs(N_samples)

                #next_sample = np.reshape(next_sample,(2,N_samples))

                cost=0
                for k in range(N_samples):
                    cost+=state_cost(next_sample[k,:],goal_points,obs_points)

                lb,ub = lb_ub_support(next_state,cov)
                q_const = q_constant(lb,ub)
                log_DKL = np.exp(-(-f.entropy())-cost/N_samples)
                pf[i,j] = target_pf*log_DKL #Calculate the DKL for each possible input, get corresponding probability
        S2 = np.sum(pf) #Normalize resulting policy
        #print(pf)
        pf = np.array([x/S2 for x in pf])
        
        flat = pf.flatten()

        sample_index = np.random.choice(a=flat.size, p=flat)

        # Take this index and adjust it so it matches the original array
        adjusted_index = np.unravel_index(sample_index, pf.shape)
        #print(adjusted_index)

        action = np.reshape(np.array([U_space_1[adjusted_index[0]],U_space_2[adjusted_index[1]]]),(2,1))
        
        return(action)
    

goal_points = np.array(np.mat('-1.4; -0.78; 0'))

obs_points = np.array(np.mat('0 0 0 0 0 -0.8;0 0.2 0.4 0.6 0.8 -0.8;0 0 0 0 0 0'))

obs_points_f = np.array(np.mat('0 0 0 0 0 0.8 0.8 0.8 0.8 0.8 -0.8 -0.8 -0.8 -0.8 -0.8;-0.8 -0.4 0 0.4 0.8 -0.8 -0.4 0 0.4 0.8 -0.8 -0.4 0 0.4 0.8;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0'))

    
# Instantiate Robotarium object
N = 1
M = 3
# initial_conditions = [np.array(np.mat('1.4;0.9; 0')),np.array(np.mat('0.2;0.9;0')),np.array(np.mat('1.2;-0.8; 0')),np.array(np.mat('-1.0;0.9; 0'))]
initial_conditions = [np.array(np.mat('1.0;0.9; 0')),np.array(np.mat('1.0;-0.8; 0')),np.array(np.mat('-1.4;0.9; 0'))]
#initial_conditions = [np.array(np.mat('-1;0.9; 0'))]
X_Si = [0]*M
D_Xi = [0]*M
X_SI_Prev= [0]*M

for I in range(M):
    
    X_si = []
    X_si_prev = []
    D_xi = []

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
    X_si_prev.append(x_si)
    
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
    #robot_markers = [r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width) 
    #for ii in range(goal_points.shape[1])]

    #Text with goal identification
    obs_caption = ['OBS{0}'.format(ii) for ii in range(obs_points.shape[1])]
    #Plot text for caption
    obs_points_text = [r.axes.text(obs_points[0,ii], obs_points[1,ii], obs_caption[ii], fontsize=font_size, color='k',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=-2)
    for ii in range(obs_points.shape[1])]
    obs_markers = [r.axes.scatter(obs_points[0,ii], obs_points[1,ii], s=marker_size_obs, marker='s', facecolors='none',edgecolors=CM[ii+1,:],linewidth=line_width,zorder=-2)
    for ii in range(obs_points.shape[1])]

    r.step()
    
        # While the number of robots at the required poses is less
    # than N...
    while (np.size(at_pose(np.vstack((x_si,x[2,:])), goal_points, position_error=0.15,rotation_error=100)) != N):

        # Get poses of agents
        x = r.get_poses()
        x_si = uni_to_si_states(x)
        #print(x_si.shape)
        X_si.append(x_si)
        X_si_prev.append(x_si)
        
        #Update Plot
        # Update Robot Marker Plotted Visualization
        #for i in range(x.shape[1]):
            #robot_markers[i].set_offsets(x[:2,i].T)
            # This updates the marker sizes if the figure window size is changed. 
            # This should be removed when submitting to the Robotarium.
            #robot_markers[i].set_sizes([determine_marker_size(r, robot_marker_size_m)])

        for j in range(goal_points.shape[1]):
            goal_markers[j].set_sizes([determine_marker_size(r, goal_marker_size_m)])
        
        for j in range(obs_points.shape[1]):
            obs_markers[j].set_sizes([determine_marker_size(r, obs_marker_size_m)])

        # Create single-integrator control inputs
        #dxi = single_integrator_position_controller(x_si, goal_points[:2][:])
        dxi = Control_step(x_si,U_space_1,U_space_2,goal_points,obs_points) 
        D_xi.append(dxi)
        #print(dxi)

        # Create safe control inputs (i.e., no collisions)
        #dxi = si_barrier_cert(dxi, x_si)

        # Transform single integrator velocity commands to unicycle
        dxu = si_to_uni_dyn(dxi, x)

        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        r.set_velocities(np.arange(N), dxu)
        # Iterate the simulation
        r.step()
        
    D_Xi[I] = D_xi
    X_Si[I] = X_si
    X_SI_Prev[I] = X_si_prev

    #Call at end of script to print debug information and for your script to run on the Robotarium server properly
    r.call_at_scripts_end()
    
D_Xi = np.array(D_Xi,dtype=object)
X_Si = np.array(X_Si,dtype=object)

np.save('New_State_Data_Simulation.npy',X_Si)
np.save('New_Input_Data_Simulation.npy',D_Xi)
np.save('New_State_Data_Simulation_Prev.npy',X_SI_Prev)
