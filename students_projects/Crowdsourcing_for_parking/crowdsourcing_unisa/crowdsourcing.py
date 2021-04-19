import math

import numpy as np  # for array managment


class Crowdsourcing():
    def __init__(self, generator_contribs, agent):
        self.contribs = generator_contribs
        self.agent = agent

    # initialing an empty array for the returned behavior (N x N_nodes)

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

    def step_route(self):
        # weights for the algorithm
        routes = []
        costs = np.zeros(shape=self.agent.n_destinations)
        for r in range(self.agent.n_destinations):
            state = self.agent.state
            routes.append([state])
            while state != self.agent.park_list['index'][r]:
                A = np.zeros(shape=self.contribs.n_contribs, dtype=float)
                for j in range(self.contribs.n_contribs):
                    dkl = self.__DKL(self.contribs.PMF_contribs[j][state], self.agent.PMF_targets[r][state])
                    exp_v = self.__expected_value(self.agent.rewards[r], self.contribs.PMF_contribs[j][state])
                    # print('Dkl', dkl)
                    # print('exp_v', exp_v)
                    A[j] = dkl - exp_v  # cost
                i_min = np.argmin(A)
                # print('A', A)
                # print('contributor', i_min)
                behav = self.contribs.PMF_contribs[i_min][state]
                # print('behavior', behav)
                new_state = np.argmax(behav)
                if new_state in routes[r]:
                    print('loop per route', r)
                    costs[r] = np.inf
                    break
                else:
                    routes[r].append(new_state)
                    costs[r] += A[i_min]
                    state = new_state
                    # if costs[r] > costs.min():
                    #     break

        route_min = costs.argmin()
        print("routes", routes)
        print("cost of the 3 routes", costs, "route chosen", route_min)
        self.agent.convert_route(routes[route_min])
