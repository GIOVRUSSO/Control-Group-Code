import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

# TODO: Delete this file before submitting the final version of the project

class Cost:
    def __init__(self, obstacles_positions, obstacles_sizes, alpha, beta):
        self.obstacles_positions = obstacles_positions
        self.obstacles_sizes = obstacles_sizes
        self.alpha = alpha
        self.beta = beta

    def get_cost_function(self):
        def cost(x, t):
            
            r = np.array([10.0, 10.0])
            #z_torso = 1.06
            costs = 50*((x[0] - r[0])**2 + (x[1] - r[1]))**2 -10/(np.sqrt(( ((x[0]-r[0])/1000)**2 + ((x[1] - r[1])/1000)**2 +0.1)))
            
            costs = np.squeeze(costs)
            #costs += self.alpha*(x[:,2] - z_torso)**2

            # obstacles are modeled as bivariate gaussian obstacles
            for i in range(len(self.obstacles_positions)):
                obs = self.obstacles_positions[i]
                size = self.beta*self.obstacles_sizes[i]
                costs += self.alpha*np.exp(-((x[0] - obs[0])**2/(2*size[0]**2) + (x[1] - obs[1])**2/(2*size[1]**2)))


            # Add wall costs. The walls are modeled as univariate gaussian obstacles from the corners of the room: x = -1, x = 11, y = -1, y = 11.
            # x_s = np.array([-1, 11])
            # y_s = np.array([-1, 11])
            # wall_size = 0.5
            # wall_alpha = self.alpha
            # wall_cost = 0
            # for x_i in x_s:
            #     wall_cost += wall_alpha*np.exp(-((x[0] - x_i)**2/(2*wall_size**2)))
            # for y_i in y_s:
            #     wall_cost += wall_alpha*np.exp(-((x[1] - y_i)**2/(2*wall_size**2)))
            # costs += wall_cost
            
            return costs
        return cost

if __name__ == "__main__":
    # plot 3d graph of cost function
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = np.linspace(-2, 12, 200)
    y = np.linspace(-2, 12, 200)
    x, y = np.meshgrid(x, y)

    # Define obstacles and other parameters
    obstacles_positions = [[3, 3],[4,4], [5, 5]]
    obstacles_sizes = [[1, 1],[1,1], [1, 1]]
    alpha = 1000.0
    beta = 10

    cost_obj = Cost(obstacles_positions, obstacles_sizes, alpha, beta)
    cost = cost_obj.get_cost_function()

    #z = cost(np.array([x.flatten(), y.flatten()]).T, 0).reshape(x.shape)
    arr = np.array([x.flatten(), y.flatten()]).T
    costs = np.zeros(arr.shape[0])
            
    for i in range(arr.shape[0]):
        costs[i] = cost(arr[i], 0)
    z = costs.reshape(x.shape)
    
    surf = ax.plot_surface(x, y, z, cmap=cm.coolwarm, linewidth=0, antialiased=False)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    fig.colorbar(surf, shrink=0.5, aspect=5)
    plt.show()

    # plot 2d graph of cost function
    fig = plt.figure()
    ax = fig.add_subplot(111)
    surf = ax.contourf(x, y, z, cmap=cm.coolwarm)
    fig.colorbar(surf, shrink=0.5, aspect=5)
    plt.show()