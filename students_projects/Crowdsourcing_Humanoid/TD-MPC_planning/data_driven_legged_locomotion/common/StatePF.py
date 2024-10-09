from abc import ABC, abstractmethod
import numpy as np

from .StateSpace import StateSpace, DiscreteStateSpace

class StatePF(ABC): #pi(x_k)
    """A StatePF is a probability distribution over the state space."""
    def __init__(self, ss: StateSpace):
        self.ss = ss
        
    @abstractmethod
    def getProb(self, state: np.ndarray) -> float:
        pass
    
    @abstractmethod
    def sample(self, num_samples: int = 1) -> np.ndarray:
        pass
    
    @abstractmethod
    def getMean(self) -> np.ndarray:
        pass
    
    @abstractmethod
    def getVariance(self) -> np.ndarray:
        pass
    
    @abstractmethod
    def getEntropy(self) -> float:
        pass
    
    def monteCarloExpectation(self, func: callable, num_samples: int = 50) -> float:
        """Computes the expectation of a function using Monte Carlo sampling."""
        #samples = np.array([self.sample() for _ in range(num_samples)])
        samples = self.sample(num_samples=num_samples)
        return np.mean(func(samples))
    
class StateCondPF(ABC): #pi(x_k|x_k-1)
    """An StateCondPF is a conditional PF that describes the behavior of a system at a given time."""
    def __init__(self, ss: StateSpace):
        self.ss = ss
    
    @abstractmethod
    def getNextStatePF(self, state: np.ndarray) -> StatePF:
        """Returns the next state probability distribution given the current state."""
        raise NotImplementedError("Method not implemented.")
    
class DiscreteStateCondPF(StateCondPF):
    """A StateCondPF is a conditional PF that describes the behavior of a system at a given time over a discrete state space."""
    def __init__(self, ss: DiscreteStateSpace):
        self.ss = ss
        self.PFs = {}
        
    @abstractmethod
    def _getNextStatePFFromIndex(self, state_index: tuple) -> StatePF:
        """Returns the next state probability distribution given the current state index."""
        pass
    
    def getNextStatePF(self, state: np.ndarray) -> StatePF:
        """Returns the next state probability distribution given the current state."""
        state_index = self.ss.toIndex(state)
        x_index_flat = np.ravel_multi_index(state_index, self.ss.dims) # array of indices (n_samples,)
        if not x_index_flat in self.PFs:
            self.PFs[x_index_flat] = self._getNextStatePFFromIndex(state_index)
        return self.PFs[x_index_flat]

class HistogramStatePF(StatePF):
    """A StatePF that represents a probability distribution over a discrete state space using a histogram."""
    def __init__(self, ss: DiscreteStateSpace, data: np.ndarray):
        super().__init__(ss)
        if np.any(data.shape != ss.dims):
            raise ValueError("Histogram must have the same shape as the state space.")
        self.histogram = data
        # Normalize the histogram
        self.histogram = self.histogram / np.sum(self.histogram)
        
    def getProb(self, state: np.ndarray) -> float:
        state_index = self.ss.toIndex(state)
        return self.histogram[state_index]
    
    def sample(self, num_samples: int = 1) -> np.ndarray:
        if num_samples != 1:
            raise ValueError("HistogramStatePF only supports sampling one state at a time.")
        index_flat = np.random.choice(np.prod(self.ss.dims), p=self.histogram.flatten())
        index = np.unravel_index(index_flat, self.ss.dims)
        index = np.array(index)
        return self.ss.toState(index)
    
    def getMean(self) -> np.ndarray:
        mean = 0
        for index, state in self.ss:
            mean = mean + state * self.getProb(state)
        return mean
    
    def getVariance(self) -> np.ndarray:
        variance = 0
        for index, state in self.ss:
            variance = variance + (state - self.getMean())**2 * self.getProb(state)
        return variance
    
    def getEntropy(self) -> float:
        entropy = 0
        for index, state in self.ss:
            prob = self.getProb(state)
            if prob > 0:
                entropy = entropy - prob * np.log(prob)
        return entropy
    
class NormalStatePF(StatePF):
    """A StatePF that represents a probability distribution over the state space using a normal distribution."""
    def __init__(self, ss: StateSpace, mean: np.ndarray, cov: np.ndarray):
        super().__init__(ss)
        self.mean = mean
        self.cov = cov
        self.inv_cov = np.linalg.inv(cov)
        self.det_cov = np.linalg.det(cov)
        
    def getProb(self, state: np.ndarray) -> float:
        return np.exp(-0.5 * (state - self.mean).T @ self.inv_cov @ (state - self.mean)) / np.sqrt((2 * np.pi)**self.ss.n_states * self.det_cov)
    
    def sample(self, num_samples: int = 1) -> np.ndarray:
        return np.random.multivariate_normal(self.mean, self.cov, size=num_samples)
    
    def getMean(self) -> np.ndarray:
        return self.mean
    
    def getVariance(self) -> np.ndarray:
        return self.cov
    
    def getEntropy(self) -> float:
        return 0.5 * np.log((2 * np.pi * np.e)**self.ss.n_states * self.det_cov)
    
class NormalStateCondPF(StateCondPF):
    """A StateCondPF that represents a normal posterior distribution over the state space."""
    def __init__(self, ss: StateSpace):
        super().__init__(ss)
        
    def get_mean_cov(self, state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError("Method not implemented.")
        
    def getNextStatePF(self, state: np.ndarray) -> StatePF:
        mean, cov = self.get_mean_cov(state)
        return NormalStatePF(self.ss, mean, cov)
    
class FakeStateCondPF(StateCondPF):
    def __init__(self, ss: StateSpace, pf: StatePF):
        super().__init__(ss)
        self.pf = pf
    
    def getNextStatePF(self, state: np.ndarray) -> StatePF:
        return self.pf

if __name__ == "__main__":
    bounds = np.array([[-1.0,1.0],[0.0,1.0]])
    deltas = [0.1,0.5]
    ss = DiscreteStateSpace(2,bounds,deltas)
    data = np.random.rand(*ss.dims)
    data = data / np.sum(data)
    hist_pf = HistogramStatePF(ss, data)
    print(hist_pf.getProb(np.array([0,0])))
    print(hist_pf.sample())
    print(hist_pf.getMean())
    print(hist_pf.getVariance())
    print(hist_pf.getEntropy())
    mean = np.array([0.0,0.5])
    cov = np.array([[0.1,0.0],[0.0,0.1]])
    norm_pf = NormalStatePF(ss, mean, cov)
    print(norm_pf.getProb(np.array([0,0])))
    print(norm_pf.sample())
    print(norm_pf.getMean())
    print(norm_pf.getVariance())
    print(norm_pf.getEntropy())
    print(norm_pf.monteCarloExpectation(lambda x: x[:,0]**2+x[:,1]**2, num_samples=10000))