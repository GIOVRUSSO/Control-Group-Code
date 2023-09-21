import numpy as np
from scipy.stats import multivariate_normal

class trajectory_based_plant():

    def get_plant(self, dim, uniform):

        trackData = np.load('/home/marco/catkin_ws/src/husky_mpc_datadriven/src/data/last_result_for_thesis/dynamic_0/klc_online_simulation_0.npy')
        self.conditional = uniform

        print(trackData[0][-1])

        print(np.shape(self.conditional))

        #Reshaping the data of the tracking
        transposed_data = np.transpose(trackData)         
        #print(np.shape(transposed_data))
        self.sysData = np.array([transposed_data])


        # Set up parameters and dimensions
        self.Zdim = dim
        self.Zmin = [0, 0]
        self.Zstep = [0.5, 0.5]
        self.Zdiscr = [19, 19]

        (full, Y) = self.getJointPMFs()
        cond = self.getConditional(full, Y)

        print(np.shape(cond))

        indices = np.where(cond != 0)
        non_zero_positions = list(zip(*indices))

        for index in non_zero_positions:
            self.conditional[index] = cond[index] + abs(np.random.normal(0, 0.05))                        

        return self.conditional


    def discretize(self, Z):
        res = [0] * self.Zdim  # n-dimensional index
        for i in range(self.Zdim):  # For each dimension
            elt = Z[i]  # Extract the i-th element
            ind = int((elt - self.Zmin[i]) // self.Zstep[i])  # Discretize
            res[i] = ind
        return tuple(res)  # Return as tuple for array indexing
    
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


"""p = trajectory_based_plant().get_plant(2, )

# Creiamo un elenco per salvare i valori non nulli
non_zero_values = []

# Itera attraverso gli elementi dell'array 'p'
for x in p:
    non_zero_values.append(x)

# Salva i valori non nulli in un file (ad esempio, 'non_zero_values.txt')
with open('non_zero_values.txt', 'w') as f:
    for value in non_zero_values:
        f.write(str(value) + '\n')"""

