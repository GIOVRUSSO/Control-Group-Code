import math

import numpy as np  #for array managment
from scipy.optimize import minimize


###The behaviors are loaded
behaviors = np.load('behaviors.npy')
S = 3


class Crowdsourcing():
    def __init__(self, agent):
        #the sources are a global variable

        self.fullDim = behaviors.shape[1]
        self.N_contribs = behaviors.shape[0]

        self.nTarget = agent.id_goal
        self.PMF_target = behaviors[self.nTarget]

        self.cons = ({'type': 'eq', 'fun': lambda x:  np.sum(x)-1})

    # initialing an empty array for the returned behavior (N x N_nodes)

    def __DKL(self, l1, l2):
        ###DKL between two arrays, they have to be pmfs
        # Behavior identical to scipy.stats.entropy for two histograms
        x = 0
        for i in range(len(l1)):
            if l1[i] != 0 and l2[i] != 0:
                x = x + l1[i] * math.log(l1[i] / l2[i])
            #if l2[i] == 0 and l1[i] != 0:
            #    return math.inf
        return x
        
    def update_target(self):
        self.PMF_target = behaviors[self.nTarget]
        
    def receding_horizon_DM(self, state, tHor, stateSpace, r):
        ### Corwdsourcing decision-making loop
        # Arguments: state, time horizon, state space, reward array
        xInd = np.where(stateSpace == state)[0][0] #Index of the state in the reduced state space
        dim = len(stateSpace)
        rHat = np.array([0]*self.fullDim) #rHat as in the algorithm
        weights = np.zeros((self.N_contribs, dim)) #Decision-making weights

        sources = np.zeros((self.N_contribs, dim, self.fullDim)) #The sources and target we use correspond to the reduced state space
        target = np.zeros((dim,self.fullDim))
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

    def max_sampling_DM(self, state, tHor, stateSpace, r):
        ### Crowdsourcing decision-making loop with maximum sampling
        # Arguments: state, time horizon, state space, reward array
        xInd = np.where(stateSpace == state)[0][0] #Index of the state in the reduced state space
        dim = len(stateSpace)
        rHat = np.array([0]*self.fullDim) #rHat as in the algorithm
        weights = np.zeros((self.N_contribs, dim)) #Decision-making weights

        sources = np.zeros((self.N_contribs, dim, self.fullDim)) #The sources and target we use correspond to the reduced state space
        target = np.zeros((dim,self.fullDim))
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
        print(weights[:,xInd])
        pf = sources[indMin, xInd] #Pick pf
        return(np.argmax(pf)) #Max-sample pf and return resulting state


    def getCost(self, weights, r, x):
        ###
        #Given a vector of weights, compute pf and corresponding cost
        pf = [0]*len(behaviors[0][x])
        for i in range(S):
            pf = pf + weights[i]*behaviors[i][x]
        return(self.__DKL(pf, self.PMF_target[x]) - np.dot(pf,r))

    def getWeights(self, r, x):
        w = minimize(self.getCost, [1/S]*S, args=(r,x), bounds = ((0,1),)*S, constraints=(self.cons)) #No constraints
        #w = minimize(self.getCost, [1/S]*S, args=(r,x), bounds = ((0,1),)*S, constraints=(self.cons, self.safety_cons(x,119), self.safety_cons(x,96))) #For road occlusion avoidance, safety constraints can be added and modified according to user specifications
        #print(w.message) #For debugging: message from the optimizer
        return(w.x)

    def merging_DM(self, state, tHor, stateSpace, r):
        ###
        #Source-merging decision-making step
        xInd = np.where(stateSpace == state)[0][0] #Index of current state
        dim = len(stateSpace)
        rHat = np.array([0.0]*self.fullDim)
        weights = np.zeros((self.N_contribs, dim))
        if tHor == 0:
            tHor = 1
    
        timeHor = list(range(tHor))
        timeHor.reverse()
    
        for t in timeHor:
            rBar = r + rHat #Reward update
            for i in range(dim):
                x = stateSpace[i]
                weights[:,i] = self.getWeights(rBar, x)
                rHat[x] = -self.getCost(weights[:,i],rBar,x)
        w = weights[:,xInd] #Get weights for current state
        pf = [0]*self.fullDim #Recreate pf
        for i in range(S):
            pf = pf + w[i]*behaviors[i][state]
        return(np.random.choice(range(self.fullDim), p = pf))

    def max_sampling_merging(self, state, tHor, stateSpace, r):
        ###
        #Max sampling variant for the HIL experiments
        xInd = np.where(stateSpace == state)[0][0]
        dim = len(stateSpace)
        rHat = np.array([0.0]*self.fullDim)
        weights = np.zeros((self.N_contribs, dim))

        if tHor == 0:
            tHor = 1
    
        timeHor = list(range(tHor))
        timeHor.reverse()
    
        for t in timeHor:
            rBar = r + rHat #Reward update
            for i in range(dim):
                x = stateSpace[i]
                weights[:,i] = self.getWeights(rBar, x)

                rHat[x] = -self.getCost(weights[:,i],rBar,x)
        w = weights[:,xInd] #Get the weights for the current state
        pf = [0]*self.fullDim #Recreate the resulting pf
        for i in range(S):
            pf = pf + w[i]*behaviors[i][state]

        return(np.argmax(pf))

    def safety_cons(self, x, bad):#0.0257
        return({'type': 'ineq', 'fun':lambda w: 0.0257 - (w[0]*behaviors[0,x] + w[1]*behaviors[1,x] + w[2]*behaviors[2,x])[bad]})
        #return({'type': 'ineq', 'fun':lambda w: - 0.94 + (w[0]*behaviors[0,x] + w[1]*behaviors[1,x] + w[2]*behaviors[2,x])[bad]})

