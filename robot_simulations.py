import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np
from scipy import stats
import cvxpy as cp
from time import time
from casadi import *
import matplotlib.pyplot as plt


def KL_div(p, q):
    # Check if both data sets have the same shape
    if p.shape != q.shape:
        raise ValueError('Both data sets must have the same shape')

    # Add a small epsilon to avoid log(0) or division by zero
    epsilon = 1e-10  # Small value to avoid log(0) and division by zero
    p = np.clip(p, epsilon, 1)  # Clip values to avoid zero
    q = np.clip(q, epsilon, 1)  # Clip values to avoid zero

    # Normalize the binned data sets to obtain probability distributions
    p = p / np.sum(p)
    q = q / np.sum(q)

    # Calculate the KL divergence
    KL_d = np.sum(p * (np.log(p) - np.log(q)))

    return KL_d


def compute_cost(x_axis, obs_points, goal_points, n_x_1, n_x_2, flag_obs, obs_size):
    # Cost definition
    # Cost for the obstacles
    cost_obs_term = np.zeros(n_x_1 * n_x_2)
    if flag_obs:
        var_o = (4 * obs_size / 2) ** 2
        for o in range(np.size(obs_points, axis=1)):
            diff_vec = np.array(
                [[x_axis[0][i1] - obs_points[0, o], x_axis[1][i2] - obs_points[1, o]] for i1 in range(n_x_1)
                 for i2 in range(n_x_2)])
            dist_vec = np.sqrt(np.sum(diff_vec ** 2, axis=1))
            cost_obs_term = cost_obs_term + np.exp(-1 / (2 * var_o) * dist_vec ** 2)

    # Cost for the distance from the goal
    diff_vec_g = np.array(
        [[x_axis[0][i1] - goal_points[0][0], x_axis[1][i2] - goal_points[1][0]] for i1 in range(n_x_1) for
         i2 in range(n_x_2)])
    cost_g = np.sqrt(np.sum(diff_vec_g ** 2, axis=1))

    # Cost for the boundaries
    var_b = 0.01 ** 2
    cost_b_term = np.zeros(n_x_1 * n_x_2)
    for b_ind in range(2):
        dist_vec_x = np.transpose(np.array([[np.abs(x_axis[0][i1] - boundary_points[b_ind]) for i1 in range(n_x_1)]]
                                           * n_x_bins2)).flatten()
        dist_vec_y = np.array([[np.abs(x_axis[1][i2] - boundary_points[b_ind + 2]) for i2 in range(n_x_2)]]
                              * n_x_1).flatten()
        cost_b_term = (cost_b_term + np.exp(-1 / (2 * var_b) * dist_vec_x ** 2) +
                       np.exp(-1 / (2 * var_b) * dist_vec_y ** 2))

    # Total cost
    total_cost = 300 * cost_obs_term + 0.5 * cost_g + 30 * cost_b_term

    return total_cost


max_u = 0.2     # max input
u_dim = 2       # dimension of the input
x_dim = 2       # dimension of the state
dt = 0.033      # Robotarium time-step

# Discretization points of the input
n_u_bins = 7
u_axis = np.array(np.linspace(-max_u, max_u, n_u_bins))

# Discretization points of the state
boundary_points = np.array([-1.6, 1.6, -1.0, 1.0])
n_x_bins1 = 33
n_x_bins2 = 21
x_axis = [[*np.linspace(boundary_points[0], boundary_points[1], n_x_bins1)],
          [*np.linspace(boundary_points[2], boundary_points[3], n_x_bins2)]]

nSims = 5

max_steps = 3000

is_obs = 1      # 1 if there are obstacles, 0 otherwise

CM = ['b', 'g', 'r', 'c', 'm']  # Colors for the trajectories plot

# Define goal points
goal_points = np.array(np.mat('-1.4; 0; 0'))
# Create Goal Point Markers
goal_marker_size_m = 0.1
# Text with goal identification
goal_caption = 'G'

# Define obstacles
obs_points = np.array(np.mat('0 -0.8 0.5;0.5 -0.1 -0.3'))
# Create Obstacle Points Markers
obs_marker_size_m = 0.08
obs_caption = [r'$O_{0}$'.format(ii) for ii in range(obs_points.shape[1])]

# initial_conditions = np.array(np.mat('1; 0.8; 0'))
initial_conditions = [np.array(np.mat('1.2; 0; 0')), np.array(np.mat('1.2; 0.6; 0')), np.array(np.mat('-0.3;0.8; 0')),
                      np.array(np.mat('-0.1; -0.6; 0')), np.array(np.mat('1; -0.7; 0'))]


# Primitives definition
N_primitives = 4      # Number of primitives
# Mean values of the input given by the primitives:
control_input = np.array([[max_u, 0],       # go right
                          [-max_u, 0],      # go left
                          [0, max_u],       # go up
                          [0, -max_u]])     # go down
cov = np.array([0.005, 0.005, 0.005, 0.005])    # Covariance of each primitive

pi_u = np.zeros((N_primitives, n_u_bins, n_u_bins))   # Initialize primitives

for i in range(N_primitives):
    pi_u[i] = stats.multivariate_normal.pdf(
        np.array([[u_axis[i1], u_axis[i2]] for i1 in range(n_u_bins) for i2 in range(n_u_bins)]),
        control_input[i], np.array([[cov[i], 0], [0, cov[i]]])).reshape(n_u_bins, n_u_bins)
    # Normalize
    pi_u[i] = pi_u[i] / np.sum(pi_u[i])


# Compute the cost
tot_cost = compute_cost(x_axis, obs_points, goal_points, n_x_bins1, n_x_bins2, is_obs, obs_marker_size_m)


# Cost Heatmap
# Create a figure and axes
fig, ax = plt.subplots(figsize=[9, 6])
plt.rcParams.update({'font.size': 14})
rotated = np.transpose(tot_cost.reshape(n_x_bins1, n_x_bins2))

# Create a heatmap using imshow()
contours = ax.contourf(x_axis[0], x_axis[1], rotated, 500, cmap='coolwarm')
#contour1 = ax.contour(x_axis[0], x_axis[1], rotated, cmap='coolwarm', linestyles='dashed')

# Add a colorbar
cbar = plt.colorbar(contours, shrink=0.8)

# Add labels and title
ax.set_xlabel('$p_x$ [m]', fontsize=14)
ax.set_ylabel('$p_y$ [m]', fontsize=14)
plt.xticks(fontsize=12)
plt.yticks(fontsize=12)

plt.savefig('robot_cost_map.png', dpi=1000)


N = 1   # Number of robots

# Initialize variables
position_history = np.zeros((nSims, 2, max_steps))  # trajectory of the robot
err = np.zeros((nSims, max_steps))                  # position error
w_h = np.zeros((nSims, max_steps, N_primitives))    # weights
step = np.zeros(nSims, dtype=int)                   # number of steps

for n_sim in range(nSims):
    # Instantiate Robotarium object
    r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions[n_sim],
                              sim_in_real_time=False)

    # Create single integrator position controller
    single_integrator_position_controller = create_si_position_controller()

    _, uni_to_si_states = create_si_to_uni_mapping()

    # Create mapping from single integrator velocity commands to unicycle velocity commands
    si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

    # Define x initially
    x = r.get_poses()
    x_si = uni_to_si_states(x)

    # Scale the plotted markers to be the diameter of provided argument (in meters)
    marker_size_goal = determine_marker_size(r, goal_marker_size_m)
    font_size = determine_font_size(r, 0.08)
    # Plot text for caption
    goal_points_text = [r.axes.text(goal_points[0], goal_points[1], goal_caption, fontsize=font_size, color='k',
                                    fontweight='bold', horizontalalignment='center', verticalalignment='center',
                                    zorder=-1)]
    goal_markers = [r.axes.scatter(goal_points[0], goal_points[1], s=marker_size_goal,
                                   facecolors='none', edgecolors='g', linewidth=5, zorder=-1)]

    if is_obs:
        marker_size_obs = determine_marker_size(r, obs_marker_size_m)
        obs_points_text = [r.axes.text(obs_points[0, ii], obs_points[1, ii], obs_caption[ii], fontsize=font_size, color='k',
                                       fontweight='bold', horizontalalignment='center', verticalalignment='center',
                                       zorder=-2)
                           for ii in range(obs_points.shape[1])]
        obs_markers = [r.axes.scatter(obs_points[0, ii], obs_points[1, ii], s=marker_size_obs, facecolors='none',
                                      edgecolors='r', linewidth=5, zorder=-2)
                       for ii in range(obs_points.shape[1])]

    r.step()
    step[n_sim] = int(0)

    print("\nStart of Simulation", n_sim + 1, "\n")

    # While the number of robots at the required poses is less than N and step[n_sim] < max_steps:
    while (np.size(at_pose(np.vstack((x_si, x[2, :])), goal_points, position_error=0.05, rotation_error=100)) != N
           and step[n_sim] < max_steps):

        # Get poses of agents
        x = r.get_poses()
        x_si = uni_to_si_states(x)  # Convert to single integrator state variables
        position_history[n_sim, :, step[n_sim]] = x_si[:2].reshape(2, )     # Save position

        # Update the marker sizes if the figure window size is changed (to be removed when submitting to the Robotarium)
        goal_markers[0].set_offsets(goal_points[:2].T)
        goal_markers[0].set_sizes([determine_marker_size(r, goal_marker_size_m)])
        if is_obs:
            for i in range(obs_points.shape[1]):
                obs_markers[i].set_offsets(obs_points[:2, i].T)
                obs_markers[i].set_sizes([determine_marker_size(r, obs_marker_size_m)])

        w = cp.Variable((1, N_primitives))  # Weights
        constraints = [w >= 0, cp.sum(w) == 1]

        KL = []     # Initialize KL divergence

        # Define q_u
        q_u = stats.multivariate_normal.pdf(
            np.array([[u_axis[i1], u_axis[i2]] for i1 in range(n_u_bins) for i2 in range(n_u_bins)]),
            single_integrator_position_controller(x_si + np.random.normal(0, 0.008, size=x_si.shape),
                                                  goal_points[:2][:]).reshape(2, ),
            np.array([[0.005, 0], [0, 0.005]])).reshape(n_u_bins, n_u_bins)
        # Normalize
        q_u = q_u / np.sum(q_u)

        el = np.zeros((N_primitives, n_u_bins, n_u_bins))

        for z1 in range(n_u_bins):
            for z2 in range(n_u_bins):
                # Define p_x
                p_x = stats.multivariate_normal.pdf(
                    np.array([[x_axis[0][i1], x_axis[1][i2]] for i1 in range(n_x_bins1) for i2 in range(n_x_bins2)]),
                    (x_si.reshape(2, ) + np.array([u_axis[z1], u_axis[z2]]) * dt),
                    np.array([[0.008, 0], [0, 0.008]]))
                # Normalize
                p_x = p_x / np.sum(p_x)

                # Define q_x
                q_x = stats.multivariate_normal.pdf(
                    np.array([[x_axis[0][i1], x_axis[1][i2]] for i1 in range(n_x_bins1) for i2 in range(n_x_bins2)]),
                    (x_si.reshape(2, ) + np.array([u_axis[z1], u_axis[z2]]) * dt),
                    np.array([[0.002, 0], [0, 0.002]]))
                # Normalize
                q_x = q_x / np.sum(q_x)

                # Define p_u as the weighted sum of the primitives
                p_u_comb = 0
                for i in range(N_primitives):
                    p_u_comb = p_u_comb + w[0, i] * pi_u[i, z1, z2]

                # KL divergence between pi_u and q_u
                KL = np.append(KL, cp.sum(cp.rel_entr(p_u_comb, q_u[z1, z2])))

                cost_x = np.sum(np.multiply(p_x, tot_cost))

                for i in range(N_primitives):
                    el[i, z1, z2] = pi_u[i, z1, z2] * (cost_x + KL_div(p_x, q_x))

        cost = 0
        for i in range(N_primitives):
            cost = cost + w[0, i] * np.sum(el[i])

        L = np.sum(KL) + cost  # sum KL over u

        # Find the optimal weights at this step k
        objective = cp.Minimize(L)
        prob = cp.Problem(objective, constraints)
        # Limit the max iterations to reduce computational time
        result = prob.solve(solver=cp.SCS, verbose=False, max_iters=1000)

        if any(w.value.reshape(N_primitives) < 0) | any(w.value.reshape(N_primitives) > 1):
            w.value = np.clip(w.value, 0, 1)  # Clip for safety

        p_comb = 0      # Initialize optimal policy
        for i in range(N_primitives):
            p_comb = p_comb + w.value[0, i] * pi_u[i]  # Linear combination of the policies with the optimal weights
        p_comb = p_comb / np.sum(p_comb)
        # Sample from the optimal policy
        u_comb_ind = np.random.choice(n_u_bins * n_u_bins, size=1, p=p_comb.flatten(order='C'))[0]
        # Sample from single primitive
        #u_comb_ind = np.random.choice(n_u_bins * n_u_bins, size=1, p=pi_u[0].flatten(order='C'))[0]
        # Sample from ref policy
        #u_comb_ind = np.random.choice(n_u_bins * n_u_bins, size=1, p=q_u.flatten(order='C'))[0]
        adjusted_index = np.unravel_index(u_comb_ind, (n_u_bins, n_u_bins), order='C')
        u_comb = np.array([u_axis[adjusted_index[0]], u_axis[adjusted_index[1]]]).reshape(2, 1)

        dxi = u_comb

        # Transform single integrator velocity commands to unicycle
        dxu = si_to_uni_dyn(dxi, x)

        # Limit the input to avoid going out of the boundary values
        radius = r.wheel_radius
        leng = r.base_length
        dxdd = np.vstack((1 / (2 * radius) * (2 * dxu[0, :] - leng * dxu[1, :]),
                          1 / (2 * radius) * (2 * dxu[0, :] + leng * dxu[1, :])))
        to_thresh = np.absolute(dxdd) > r.max_wheel_velocity
        dxdd[to_thresh] = (r.max_wheel_velocity - 0.001) * np.sign(dxdd[to_thresh])
        dxu = np.vstack((radius / 2 * (dxdd[0, :] + dxdd[1, :]),
                         radius / leng * (dxdd[1, :] - dxdd[0, :])))

        # Set the velocities by mapping the single-integrator inputs to unicycle inputs
        r.set_velocities(np.arange(N), dxu)
        # Iterate the simulation
        r.step()

        w_h[n_sim, step] = w.value[0, :]    # save weights

        step[n_sim] = step[n_sim] + 1       # update step

        # If the robot goes out of the boundaries stop the simulation
        if (x_si[0] < boundary_points[0] or x_si[0] > boundary_points[1] 
                or x_si[1] < boundary_points[2] or x_si[1] > boundary_points[3]):
            print("Robot went out of the boundaries!")
            break

    # Error = distance from the goal point
    err[n_sim] = np.sqrt(np.sum((position_history[n_sim, :, :] - goal_points[:2]) ** 2, axis=0))

    print("\nEnd of Simulation", n_sim + 1, "\n_______________________________________________")

    # Plot error and weights
    T = (step[n_sim] - 1) * dt
    fig, ax = plt.subplots(2, 1, figsize=(8, 8))
    plt.subplots_adjust(hspace=0.4)
    #plt.rcParams["text.usetex"] = True
    plt.suptitle('Simulation %i' % (n_sim+1), fontsize=16)
    ax[0].plot(np.linspace(0, T, step[n_sim]), err[n_sim, 0:step[n_sim]])
    ax[0].grid()
    ax[0].set_xlabel('time [s]')
    ax[0].set_ylabel('$e(t)$')
    ax[0].set_title('Distance from the goal', fontsize=15)
    ax[0].set_xlim(0, T)
    ax[1].plot(np.linspace(0, T, step[n_sim]), w_h[n_sim, 0:step[n_sim]])
    ax[1].grid()
    ax[1].set_xlabel('time [s]')
    ax[1].set_ylabel('$w_i(t)$')
    ax[1].set_title('Weights', fontsize=15)
    ax[1].set_xlim(0, T)
    ax[1].legend(['$w_1$', '$w_2$', '$w_3$', '$w_4$', '$w_5$'], loc="upper right", fontsize=12)
    plt.show()
    plt.savefig('results_simulation%i.png' % (n_sim+1))

    r.call_at_scripts_end()


# Plot trajectories
plt.figure()
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=np.array(np.mat('2;2;2')),
                          sim_in_real_time=True)

# Plot text for caption
goal_points_text = [r.axes.text(goal_points[0], goal_points[1], goal_caption, fontsize=font_size, color='k',
                                fontweight='bold', horizontalalignment='center', verticalalignment='center',
                                zorder=-1)]
goal_markers = [r.axes.scatter(goal_points[0], goal_points[1], s=marker_size_goal,
                               facecolors='none', edgecolors='g', linewidth=5, zorder=-1)]
if is_obs:
    # Create Obstacle Points Markers
    obs_marker_size_m = 0.08
    marker_size_obs = determine_marker_size(r, obs_marker_size_m)
    obs_caption = [r'$O_{0}$'.format(ii) for ii in range(obs_points.shape[1])]
    obs_points_text = [r.axes.text(obs_points[0, ii], obs_points[1, ii], obs_caption[ii], fontsize=font_size, color='k',
                                   fontweight='bold', horizontalalignment='center', verticalalignment='center', zorder=-2)
                       for ii in range(obs_points.shape[1])]
    obs_markers = [r.axes.scatter(obs_points[0, ii], obs_points[1, ii], s=marker_size_obs, facecolors='none',
                                  edgecolors='r', linewidth=5, zorder=-2)
                   for ii in range(obs_points.shape[1])]

for n_sim in range(nSims):
    # Plot the path
    start_m_size = determine_marker_size(r, 0.02)
    r.axes.scatter(position_history[n_sim, 0, 0], position_history[n_sim, 1, 0], s=start_m_size, linewidth=0.5,
                   color='k',
                   marker='X')
    r.axes.scatter(position_history[n_sim, 0, 1:step[n_sim]], position_history[n_sim, 1, 1:step[n_sim]], s=1,
                   linewidth=1, color=CM[n_sim], linestyle='solid')

plt.savefig('trajectories.png')  # save figure with the complete trajectories

# Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()