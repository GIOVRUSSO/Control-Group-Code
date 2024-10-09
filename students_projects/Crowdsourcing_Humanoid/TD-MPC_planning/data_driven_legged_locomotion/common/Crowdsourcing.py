from abc import ABC, abstractmethod
import cvxpy as cp
import numpy as np

from .ServiceSet import ServiceSet, BehaviorSet, Behavior
from .StatePF import StatePF, StateCondPF
from .StateSpace import StateSpace

class CrowdsourcingBase(ABC):
    """If N==1 the state space can be continuous, otherwise it must be discrete."""
    def __init__(self, ss: StateSpace, services: ServiceSet, cost: callable, N: int = 1, n_samples: int = 100):
        self.ss = ss
        self.services = services
        self.cost = cost
        self.N = N
        self.initialized = False
        self.n_samples = n_samples
        
    @property
    def S(self):
        return len(self.services)
        
    def initialize(self, initial_state: np.ndarray, time: float = 0.0):
        self._a = np.zeros((self.N + 1, self.S, self.ss.total_combinations))
        self._alpha = np.zeros((self.N + 1, self.S))
        #self._overline_r = np.zeros((self.N, self.ss.total_combinations))
        self._behaviors: BehaviorSet = self.services.getBehaviors(initial_state, self.N, time)
        self._initial_state = initial_state
        self.initialized = True
    
    @abstractmethod
    def _get_DKL(self, pi: StatePF) -> float:
        pass
    
    def _solveOptimization(self, a: np.ndarray):
        alpha = cp.Variable(self.S)
        constraints = [alpha >= 0, cp.sum(alpha) == 1]
        objective = cp.Maximize(cp.sum(cp.multiply(a, alpha)))
        problem = cp.Problem(objective, constraints)
        problem.solve()
        return alpha.value
    
    def _state_iteration(self, state: np.ndarray, k: int, behaviors: list[StateCondPF], eval_r_overline: callable):
        x_index = self.ss.toIndex(state)
        x_index_flat = np.ravel_multi_index(x_index, self.ss.dims)
        #r_hat = - np.dot(self._a[k+1, :, x_index_flat], self._alpha[k+1, :])
        #self._overline_r[k, x_index_flat] = r_hat - self.cost(x,k)
        for s in range(self.S):
            pi_cond = behaviors[s]
            pi = pi_cond.getNextStatePF(state)
            exp_r_overline = pi.monteCarloExpectation(eval_r_overline, self.n_samples)
            self._a[k, s, x_index_flat] = self._get_DKL(pi) + exp_r_overline
        self._alpha[k,:] = self._solveOptimization(self._a[k, :, x_index_flat])
    
    def _generate_eval_r_overline(self, k: int):
        def eval_r_overline(states):
            x_indexes = self.ss.toIndex(states)
            x_indexes_flat = np.ravel_multi_index(x_indexes, self.ss.dims)
            return - np.dot(self._a[k+1, :, x_indexes_flat], self._alpha[k+1, :]) - self.cost(states,k)
        return eval_r_overline
    
    def run(self) -> tuple[list[int], Behavior]:
        if not self.initialized:
            raise ValueError("Crowdsourcing must be initialized first.")
        for k in range(self.N-1, 0-1, -1): # From N-1 to 0
            # def eval_r_overline(states):
            #     x_indexes = self.ss.toIndex(states) # Tuple of ndarrays (n_states, n_samples)
            #     x_indexes_flat = np.ravel_multi_index(x_indexes, self.ss.dims) # array of indices (n_samples,)
            #     return - np.dot(self._a[k+1, :, x_indexes_flat], self._alpha[k+1, :]) - self.cost(states,k)
            eval_r_overline = self._generate_eval_r_overline(k)
            behaviors = self._behaviors.getAtTime(k)
            # When we are at the end of the recursion, we don't need to evaluate the optimal policy from any state but the initial state
            if k == 0:
                x = self._initial_state
                self._state_iteration(x, k, behaviors, eval_r_overline)
                break
            for x_index, x in self.ss:
                self._state_iteration(x, k, behaviors, eval_r_overline)
        services_sequence = np.argmax(self._alpha[0:-1], axis=1)
        return services_sequence, self._behaviors.extractBehavior(services_sequence)
      
class MaxEntropyCrowdsouring(CrowdsourcingBase):
    def __init__(self, ss: StateSpace, services: ServiceSet, cost: callable, N: int = 1, n_samples: int = 100):
        super().__init__(ss, services, cost, N, n_samples=n_samples)
    
    def _get_DKL(self, pi: StatePF) -> float:
        return -pi.getEntropy()
    
class GreedyMaxEntropyCrowdsouring(MaxEntropyCrowdsouring):
    def __init__(self, ss: StateSpace, services: ServiceSet, cost: callable, n_samples: int = 100):
        super().__init__(ss, services, cost, N=1, n_samples=n_samples) # N=1 because we are using a greedy approach
    
    def initialize(self, initial_state: np.ndarray, time: float = 0.0):
        self._a = np.zeros((self.N + 1, self.S))
        self._alpha = np.zeros((self.N + 1, self.S))
        #self._overline_r = np.zeros((self.N, self.ss.total_combinations))
        self._behaviors: BehaviorSet = self.services.getBehaviors(initial_state, self.N, time)
        self._initial_state = initial_state
        self.initialized = True
        
    def _state_iteration(self, state: np.ndarray, k: int, behaviors: list[StateCondPF], eval_r_overline: callable):
        for s in range(self.S):
            pi_cond = behaviors[s]
            pi = pi_cond.getNextStatePF(state)
            exp_r_overline = pi.monteCarloExpectation(eval_r_overline, self.n_samples)
            self._a[k, s] = self._get_DKL(pi) + exp_r_overline
        self._alpha[k,:] = self._solveOptimization(self._a[k, :])
        
    def _generate_eval_r_overline(self, k: int):
        def eval_r_overline(states):
            return - np.dot(self._a[k+1, :], self._alpha[k+1, :]) - self.cost(states,k)
        return eval_r_overline