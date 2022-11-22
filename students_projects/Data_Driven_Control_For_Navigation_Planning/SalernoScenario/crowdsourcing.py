import math
import numpy as np  # for array managment

# SALERNO NET ------------
behaviors_name = r"salerno_net/salerno_behaviors.npy"


###This is where the behaviors are loaded
behaviors = np.load(behaviors_name) 

class Crowdsourcing():
    def __init__(self, agent):
        #the sources are a global variable

        self.fullDim = behaviors.shape[1]
        self.N_contribs = behaviors.shape[0]
        self.nTarget = agent.id_goal 
        self.PMF_target = behaviors[self.nTarget]


    def __DKL(self, l1, l2):
        ###DKL between two arrays, they have to be pdfs
        # behavior identical to scipy.stats.entropy, at least for pdfs in this example's format
        # return entropy(l1, l2)
        x = 0
        for i in range(len(l1)):
            if l1[i] != 0 and l2[i] != 0:
                x = x + l1[i] * math.log(l1[i] / l2[i])
            if l2[i] == 0 and l1[i] != 0:
                return math.inf
        return x
        
        
    def receding_horizon_DM(self, state, tHor, stateSpace, r):
        ### Decision-making loop
        # Arguments: state, time horizon, state space, reward array
        xInd = np.where(stateSpace == state)[0][0] #Index of the state in the reduced state space
        dim = len(stateSpace)
        rHat = np.array([0]*self.fullDim) #rHat as in the algorithm 
        weights = np.zeros((self.N_contribs, dim)) #Decision-making weights

        sources = np.zeros((self.N_contribs, dim, self.fullDim)) #The sources and target we use correspond to the reduced state space
        target = np.zeros((dim,self.fullDim))
        
        self.PMF_target = behaviors[self.nTarget]
        for i in range(dim):
            target[i] = self.PMF_target[stateSpace[i]]


        for i in range(self.N_contribs):
            for j in range(dim):
                sources[i,j] = behaviors[i,stateSpace[j]]
        if tHor == 0: #Safety as python doesn't like range(0)
            tHor = 1
    
        timeHor = list(range(tHor))
        timeHor.reverse()
    
        for t in timeHor:
            rBar = r + rHat #Adapt reward
            for i in range(self.N_contribs):
                for j in range(dim):
                    weights[i,j] = self.__DKL(sources[i,j], target[j]) - np.dot(sources[i,j], rBar) #calculate weights
            for i in range(dim):
                rHat[stateSpace[i]] = -min(weights[:,i]) #Calculate rHat
        indMin = np.argmin(weights[:,xInd])
        pf = sources[indMin, xInd] #Pick pf
        return(np.random.choice(range(self.fullDim), p = pf)) #Sample pf and return resulting state