from abc import ABC, abstractmethod
import numpy as np

from .StateSpace import StateSpace, DiscreteStateSpace

class StatePF(ABC): #pi(x_k)
    """A StatePF is a probability distribution over the state space."""
    
    def __init__(self, ss: StateSpace):
        """
        Initializes the StatePF class.

        Args:
            ss (StateSpace): The state space.
        """
        self.ss = ss
        
    @abstractmethod
    def getProb(self, state: np.ndarray) -> float:
        """
        Returns the probability of the given state.

        Args:
            state (np.ndarray): The state for which to get the probability.

        Returns:
            float: The probability of the given state.
        """
        pass
    
    @abstractmethod
    def sample(self, num_samples: int = 1) -> np.ndarray:
        """
        Samples states from the probability distribution.

        Args:
            num_samples (int, optional): The number of samples to generate. Defaults to 1.

        Returns:
            np.ndarray: The sampled states.
        """
        pass
    
    @abstractmethod
    def getMean(self) -> np.ndarray:
        """
        Returns the mean of the probability distribution.

        Returns:
            np.ndarray: The mean of the probability distribution.
        """
        pass
    
    @abstractmethod
    def getVariance(self) -> np.ndarray:
        """
        Returns the variance of the probability distribution.

        Returns:
            np.ndarray: The variance of the probability distribution.
        """
        pass
    
    @abstractmethod
    def getEntropy(self) -> float:
        """
        Returns the entropy of the probability distribution.

        Returns:
            float: The entropy of the probability distribution.
        """
        pass
    
    def monteCarloExpectation(self, func: callable, num_samples: int = 50) -> float:
        """
        Computes the expectation of a function using Monte Carlo sampling.

        Args:
            func (callable): The function to compute the expectation of.
            num_samples (int, optional): The number of samples to use. Defaults to 50.

        Returns:
            float: The computed expectation.
        """
        samples = self.sample(num_samples=num_samples)
        return np.mean(func(samples))
    
class StateCondPF(ABC): #pi(x_k|x_k-1)
    """A StateCondPF is a conditional PF that describes the behavior of a system at a given time."""
    
    def __init__(self, ss: StateSpace):
        """
        Initializes the StateCondPF class.

        Args:
            ss (StateSpace): The state space.
        """
        self.ss = ss
    
    @abstractmethod
    def getNextStatePF(self, state: np.ndarray) -> StatePF:
        """
        Returns the next state probability distribution given the current state.

        Args:
            state (np.ndarray): The current state.

        Returns:
            StatePF: The next state probability distribution.
        """
        raise NotImplementedError("Method not implemented.")
    
class DiscreteStateCondPF(StateCondPF):
    """A StateCondPF that describes the behavior of a system at a given time over a discrete state space."""
    
    def __init__(self, ss: DiscreteStateSpace):
        """
        Initializes the DiscreteStateCondPF class.

        Args:
            ss (DiscreteStateSpace): The discrete state space.
        """
        self.ss = ss
        self.PFs = {}
        
    @abstractmethod
    def _getNextStatePFFromIndex(self, state_index: tuple) -> StatePF:
        """
        Returns the next state probability distribution given the current state index.

        Args:
            state_index (tuple): The current state index.

        Returns:
            StatePF: The next state probability distribution.
        """
        pass
    
    def getNextStatePF(self, state: np.ndarray) -> StatePF:
        """
        Returns the next state probability distribution given the current state.

        Args:
            state (np.ndarray): The current state.

        Returns:
            StatePF: The next state probability distribution.
        """
        state_index = self.ss.toIndex(state)
        x_index_flat = np.ravel_multi_index(state_index, self.ss.dims) # array of indices (n_samples,)
        if not x_index_flat in self.PFs:
            self.PFs[x_index_flat] = self._getNextStatePFFromIndex(state_index)
        return self.PFs[x_index_flat]

class HistogramStatePF(StatePF):
    """A StatePF that represents a probability distribution over a discrete state space using a histogram."""
    
    def __init__(self, ss: DiscreteStateSpace, data: np.ndarray):
        """
        Initializes the HistogramStatePF class.

        Args:
            ss (DiscreteStateSpace): The discrete state space.
            data (np.ndarray): The histogram data representing the probability distribution.

        Raises:
            ValueError: If the histogram shape does not match the state space dimensions.
        """
        super().__init__(ss)
        if np.any(data.shape != ss.dims):
            raise ValueError("Histogram must have the same shape as the state space.")
        self.histogram = data
        # Normalize the histogram
        self.histogram = self.histogram / np.sum(self.histogram)
        
    def getProb(self, state: np.ndarray) -> float:
        """
        Returns the probability of the given state.

        Args:
            state (np.ndarray): The state for which to get the probability.

        Returns:
            float: The probability of the given state.
        """
        state_index = self.ss.toIndex(state)
        return self.histogram[state_index]
    
    def sample(self, num_samples: int = 1) -> np.ndarray:
        """
        Samples states from the histogram.

        Args:
            num_samples (int, optional): The number of samples to generate. Defaults to 1.

        Returns:
            np.ndarray: The sampled states.

        Raises:
            ValueError: If num_samples is not 1.
        """
        if num_samples != 1:
            raise ValueError("HistogramStatePF only supports sampling one state at a time.")
        index_flat = np.random.choice(np.prod(self.ss.dims), p=self.histogram.flatten())
        index = np.unravel_index(index_flat, self.ss.dims)
        index = np.array(index)
        return self.ss.toState(index)
    
    def getMean(self) -> np.ndarray:
        """
        Returns the mean of the probability distribution.

        Returns:
            np.ndarray: The mean of the probability distribution.
        """
        mean = 0
        for index, state in self.ss:
            mean = mean + state * self.getProb(state)
        return mean
    
    def getVariance(self) -> np.ndarray:
        """
        Returns the variance of the probability distribution.

        Returns:
            np.ndarray: The variance of the probability distribution.
        """
        variance = 0
        for index, state in self.ss:
            variance = variance + (state - self.getMean())**2 * self.getProb(state)
        return variance
    
    def getEntropy(self) -> float:
        """
        Returns the entropy of the probability distribution.

        Returns:
            float: The entropy of the probability distribution.
        """
        entropy = 0
        for index, state in self.ss:
            prob = self.getProb(state)
            if prob > 0:
                entropy = entropy - prob * np.log(prob)
        return entropy
    
class NormalStatePF(StatePF):
    """A StatePF that represents a probability distribution over the state space using a normal distribution."""
    
    def __init__(self, ss: StateSpace, mean: np.ndarray, cov: np.ndarray):
        """
        Initializes the NormalStatePF class.

        Args:
            ss (StateSpace): The state space.
            mean (np.ndarray): The mean of the normal distribution.
            cov (np.ndarray): The covariance matrix of the normal distribution.
        """
        super().__init__(ss)
        self.mean = mean
        self.cov = cov
        self.inv_cov = np.linalg.inv(cov)
        self.det_cov = np.linalg.det(cov)
        
    def getProb(self, state: np.ndarray) -> float:
        """
        Returns the probability of the given state.

        Args:
            state (np.ndarray): The state for which to get the probability.

        Returns:
            float: The probability of the given state.
        """
        return np.exp(-0.5 * (state - self.mean).T @ self.inv_cov @ (state - self.mean)) / np.sqrt((2 * np.pi)**self.ss.n_states * self.det_cov)
    
    def sample(self, num_samples: int = 1) -> np.ndarray:
        """
        Samples states from the normal distribution.

        Args:
            num_samples (int, optional): The number of samples to generate. Defaults to 1.

        Returns:
            np.ndarray: The sampled states.
        """
        return np.random.multivariate_normal(self.mean, self.cov, size=num_samples)
    
    def getMean(self) -> np.ndarray:
        """
        Returns the mean of the normal distribution.

        Returns:
            np.ndarray: The mean of the normal distribution.
        """
        return self.mean
    
    def getVariance(self) -> np.ndarray:
        """
        Returns the covariance matrix of the normal distribution.

        Returns:
            np.ndarray: The covariance matrix of the normal distribution.
        """
        return self.cov
    
    def getEntropy(self) -> float:
        """
        Returns the entropy of the normal distribution.

        Returns:
            float: The entropy of the normal distribution.
        """
        return 0.5 * np.log((2 * np.pi * np.e)**self.ss.n_states * self.det_cov)
    
class NormalStateCondPF(StateCondPF):
    """A StateCondPF that represents a normal posterior distribution over the state space."""
    
    def __init__(self, ss: StateSpace):
        """
        Initializes the NormalStateCondPF class.

        Args:
            ss (StateSpace): The state space.
        """
        super().__init__(ss)
        
    def get_mean_cov(self, state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Returns the mean and covariance matrix for the next state given the current state.

        Args:
            state (np.ndarray): The current state.

        Returns:
            tuple[np.ndarray, np.ndarray]: The mean and covariance matrix for the next state.
        """
        raise NotImplementedError("Method not implemented.")
        
    def getNextStatePF(self, state: np.ndarray) -> StatePF:
        """
        Returns the next state probability distribution given the current state.

        Args:
            state (np.ndarray): The current state.

        Returns:
            StatePF: The next state probability distribution.
        """
        mean, cov = self.get_mean_cov(state)
        return NormalStatePF(self.ss, mean, cov)
    
class FakeStateCondPF(StateCondPF):
    """A StateCondPF that returns a fixed state probability distribution regardless of the current state."""
    
    def __init__(self, ss: StateSpace, pf: StatePF):
        """
        Initializes the FakeStateCondPF class.

        Args:
            ss (StateSpace): The state space.
            pf (StatePF): The fixed state probability distribution.
        """
        super().__init__(ss)
        self.pf = pf
    
    def getNextStatePF(self, state: np.ndarray) -> StatePF:
        """
        Returns the fixed state probability distribution regardless of the current state.

        Args:
            state (np.ndarray): The current state.

        Returns:
            StatePF: The fixed state probability distribution.
        """
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