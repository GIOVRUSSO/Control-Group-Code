# Crowdsourcing class for implementation of the crowdsourcing algorithm using offline-built behaviours

import math
import matplotlib.pyplot as plt
import numpy as np
from behaviours_maker import index2alg

# UNISA NET ------------
print('loading file')
behaviors_name = r"behaviors_Unisa_4_wip.npy" # change the name of the file according to the running simulation

behaviors = np.load(behaviors_name) # load behaviours file

class Crowdsourcing():
    def __init__(self, agent, numalgs):
        self.edgeNum = behaviors.shape[1] # number of traversable edges in the map
        self.targetNum = behaviors.shape[0] # number of targets multiplied by the number of behaviours for each target
        self.targetIndex = agent.id_goal # index of Dijkstra-related behaviour for the Agent target
        self.PMF_target = behaviors[self.targetIndex] # optimal behaviour to follow
        self.numalgs = numalgs # number of behaviours for each target

    # method for calculating Kullback-Leibler divergence between pmfs, inputs are:
    # - Crowdsourcing instance,
    # - first pmf for calculating DKL,
    # - second pmf for calculating DKL
    def __DKL(self, l1, l2):
        x = 0
        for i in range(len(l1)):
            if l1[i] != 0 and l2[i] != 0: # absolute continuity is respected
                x = x + l1[i] * math.log(l1[i] / l2[i])
            if l2[i] == 0 and l1[i] != 0: # absolute continuity is not respected
                return math.inf
        return x
        
    # method for decision-making of the next discrete state towards which moving the agent, inputs are:
    # - Crowdsourcing instance,
    # - discrete state of the agent (index of edge on which the car is),
    # - time horizon for solving the crowdsourcing problem,
    # - list of edges taken into account for decision making (indexes of the neighbouring edges around a number of crossings equal to the time horizon),
    # - array of rewards,
    # - flag to define if online behaviours must be taken into account (optional),
    # - data structure containing online-built behaviours (optional),
    # - list of traversable edges (used for debugging)(optional)
    def receding_horizon_DM(self, state, tHor, stateSpace, r, online = False, behaviours = None, edgelist=None):
        xInd = np.where(stateSpace == state)[0][0] #Index of the state in the reduced state space
        dim = len(stateSpace)
        rHat = np.array([0]*self.edgeNum) #rHat as in the algorithm 
        weights = np.zeros((self.numalgs, dim)) #Decision-making weights
        sources = np.zeros((self.numalgs, dim, self.edgeNum)) #The sources and target we use correspond to the reduced state space
        target = np.zeros((dim,self.edgeNum))
        self.PMF_target = behaviors[self.targetIndex]
        # build target behaviour
        for i in range(dim):
            target[i] = self.PMF_target[stateSpace[i]]
        # build behaviour sources
        for i in range(self.numalgs):
            for j in range(dim):
                sources[i,j] = behaviors[i+self.targetIndex,stateSpace[j]]
        
        if tHor == 0: #Safety as python doesn't like range(0)
            tHor = 1
    
        timeHor = list(range(tHor))
        timeHor.reverse()
        verbose = False # set to True for debugging
        for t in timeHor:
            if verbose:
                print('time '+str(t))
            rBar = r + rHat #Adapt reward
            if verbose:
                print([x for x in rBar if x!=0])
            for i in range(self.numalgs):
                for j in range(dim):
                    weights[i,j] = self.__DKL(sources[i,j], target[j]) - np.dot(sources[i,j], rBar) #calculate weights
            for i in range(dim):
                rHat[stateSpace[i]] = -min(weights[:,i]) #Calculate rHat
                if verbose:
                    print('rHat')
                    print([x for x in rHat if x!=0])
                    print('weights')
                    print([x for x in weights[:,i] if x!=0])
        indMin = np.argmin(weights[:,xInd])
        if verbose:
            print('edge is '+str(edgelist[stateSpace[xInd]]))
        pf = sources[indMin, xInd] #Pick pf
        if verbose:
            print([x for x in pf if x!=0])
        return(np.random.choice(range(self.edgeNum), p = pf)),indMin # return the next state selected according to the chosen behaviour, also return the index of the behaviour