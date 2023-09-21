"""
linearized_plant: A class representing a linearized plant model.

This class defines a linearized plant model that computes conditional probabilities based on a given dataset.
"""

import numpy as np
from scipy.stats import multivariate_normal

class linearized_plant:

    """
    Generate the conditional probability matrix for the linearized plant model.
    
    :param self: The instance of the class.
    :param dim: The dimensionality of the state space.
    :return: The conditional probability matrix.
    """
    def get_plant(self, dim):

        # Load system data from a file
        self.sysData = np.load('/home/marco/catkin_ws/src/husky_mpc_datadriven/src/data/3TypeSimulation.npy')

        # Set up parameters and dimensions
        self.Zdim = dim
        self.Zmin = [0, 0] 
        self.Zstep = [0.5, 0.5]
        self.Zdiscr = [21, 21]

        # Compute the joint and conditional probability matrices
        (full, Y) = self.getJointPMFs()
        cond = self.getConditional(full, Y)

        return cond

    """
    Discretize the continuous state space.
    
    :param Z: Continuous state variables.
    :return: Discretized state variables.
    """
    def discretize(self, Z):
        res = [0] * self.Zdim  # n-dimensional index
        for i in range(self.Zdim):  # For each dimension
            elt = Z[i]  # Extract the i-th element
            ind = int((elt - self.Zmin[i]) // self.Zstep[i])  # Discretize
            res[i] = ind
        return tuple(res)  # Return as tuple for array indexing

    """
    Calculate the joint and marginal probability matrices.
    
    :param self: The instance of the class.
    :return: Joint and marginal probability matrices.
    """
    def getJointPMFs(self):

        fullJoint = np.zeros(self.Zdiscr * 2)  # p(Z,Y)
        Yjoint = np.zeros(self.Zdiscr)  # p(Y)
        for Zhist in self.sysData:  # For each trajectory in the dataset
            for i in range(len(Zhist) - 1):  # For each data point in the trajectory

                Z = Zhist[i + 1]  # Extract the realization of Z and Y
                Y = Zhist[i]

                Zind = self.discretize(Z)  # Find the indexes

                Yind = self.discretize(Y)

                fullInd = Yind + Zind  # Get the index of the joint variable Z,Y
                
                fullJoint[fullInd] = fullJoint[fullInd] + 1  # Update the values
                Yjoint[Yind] = Yjoint[Yind] + 1
        fullJoint = fullJoint / np.sum(fullJoint)  # Normalize
        Yjoint = Yjoint / np.sum(Yjoint)

        return fullJoint, Yjoint

    """
    Calculate the conditional probability matrix.
    
    :param self: The instance of the class.
    :param fullJoint: Joint probability matrix.
    :param Yjoint: Marginal probability matrix for Y.
    :return: Conditional probability matrix.
    """
    def getConditional(self, fullJoint, Yjoint):

        fullDiscr = 2 * self.Zdiscr
        conditional = np.zeros(fullDiscr)  # Initialize the pmf
        for (index, x) in np.ndenumerate(fullJoint):  # For each index and value in p(Z,Y)
            Yind = index[:self.Zdim]  # Extract the index for Y
            if Yjoint[Yind] == 0:  # Protect from dividing by zero
                conditional[index] = 0
            else:
                conditional[index] = fullJoint[index] / Yjoint[Yind]  # Division
        return conditional


p = linearized_plant().get_plant(2)