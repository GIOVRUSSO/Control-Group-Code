import math

import numpy as np  # for array managment


class Crowdsourcing():
    def __init__(self, N_contribs, PMFs_contribs, reward, PMF_target):
        self.N_contribs = N_contribs
        self.PMFs_contribs = PMFs_contribs
        self.reward = reward
        self.PMF_target = PMF_target

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
        # # print ("entropia", entropy(l1,l2))
        # # print("calcolata", x)
        return x
        # return (np.sum([l1[i] * math.log2(l1[i] / l2[i]) for i in range(len(l1)) if l1[i] != 0]))

    def __expected_value(self, x, p_x):
        sum = 0
        for i in range(len(x)):
            sum = sum + x[i] * p_x[i]
        return sum

    def step_route(self, state):
        # weights for the algorithm
        A = [0.0, 0.0, 0.0]
        for j in range(self.N_contribs):
            dkl = self.__DKL(self.PMFs_contribs[j][state], self.PMF_target[state])
            exp_v = self.__expected_value(self.reward, self.PMFs_contribs[j][state])
            A[j] = dkl - exp_v
        i_min = np.argmin(A)
        # print(A)
        # print(i_min)
        behav = self.PMFs_contribs[i_min][state]
        new_state = np.argmax(behav)
        # print(new_state)
        return new_state, behav
