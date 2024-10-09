import numpy as np

class StateSpace:
    """StateSpace represents a generic continuous state space over which PFs are defined."""
    
    def __init__(self, n_states: int, bounds: np.array = None):
        """
        Initializes the StateSpace class.

        Args:
            n_states (int): The number of states.
            bounds (np.array, optional): The bounds for each state. Defaults to None.

        Raises:
            ValueError: If the dimension vector does not match the size of the ranges vector.
            ValueError: If any of the bounds are invalid.
        """
        self.n_states = n_states
        if bounds is None:
            bounds = np.repeat(np.array([[-np.inf, np.inf]]), n_states, axis=0)
        elif len(bounds) != n_states:
            raise ValueError("Dimension vector must have the same size of the ranges vector")
        for i in range(n_states):
            if bounds[i,0] > bounds[i,1]:
                raise ValueError(f"Bounds {bounds[i]} are invalid.")
        self.bounds = bounds

class DiscreteStateSpace(StateSpace):
    """DiscreteStateSpace provides a representation for a discrete state space over which PFs are defined."""
    
    def __init__(self, n_states: int, bounds: np.ndarray, max_deltas: list[int]):
        """
        Initializes the DiscreteStateSpace class.

        Args:
            n_states (int): The number of states.
            bounds (np.ndarray): The bounds for each state.
            max_deltas (list[int]): The maximum cell size in each dimension.

        Raises:
            ValueError: If bounds are not defined.
        """
        if bounds is None:
            raise ValueError("Bounds must be defined for a discrete state space.")
        super().__init__(n_states, bounds)
        self._buildStateSpace(max_deltas)

    def __iter__(self):
        """
        Initializes the iterator.

        Returns:
            DiscreteStateSpace: The iterator object.
        """
        self._iter_state = 0
        return self

    def __next__(self):
        """
        Returns the next state in the iteration.

        Returns:
            tuple: The index and the corresponding state.

        Raises:
            StopIteration: If the iteration exceeds the maximum index.
        """
        max_index = np.prod(self.dims) - 1
        if self._iter_state > max_index:
            raise StopIteration
        index = np.unravel_index(self._iter_state, self.dims)
        self._iter_state += 1
        return (index, self.toState(index))

    @property
    def ranges(self):
        """
        Returns the ranges for each state.

        Returns:
            np.ndarray: The ranges for each state.
        """
        return self.bounds[:,1] - self.bounds[:,0]
    
    @property
    def total_combinations(self):
        """
        Returns the total number of combinations in the state space.

        Returns:
            int: The total number of combinations.
        """
        return np.prod(self.dims)

    def _buildStateSpace(self, max_deltas):
        """
        Builds the state space with the given maximum deltas.

        Args:
            max_deltas (list[int]): The maximum cell size in each dimension.
        """
        self.dims = self.ranges / max_deltas # Element-wise division
        self.dims = np.ceil(self.dims).astype(int) + 1 # If the division is not an integer, we increase the resolution to make the space uniform
        self.deltas = self.ranges / (self.dims-1) # The actual sampling resolution

    def toIndex(self, state: np.ndarray) -> tuple:
        """
        Converts a state to an index in the state space.

        Args:
            state (np.ndarray): The state to convert.

        Returns:
            tuple: The index corresponding to the state.

        Raises:
            ValueError: If the state shape does not match the expected shape.
            ValueError: If the state is lower than the lower bounds.
            ValueError: If the state is higher than the upper bounds.
        """
        if len(state.shape) == 1:
            state = np.expand_dims(state, axis=0)
        if state.shape[1] != self.n_states:
            raise ValueError(f"Axis 1 has size {state.shape[1]} but {self.n_states} is expected.")
        if np.any(state < self.bounds[:,0]):
            i = state < self.bounds[:,0]
            print(i.shape)
            raise ValueError(f"State {state} is lower than the lower bounds {self.bounds[:,0]}.")
        if np.any(state > self.bounds[:,1]):
            i = state > self.bounds[:,1]
            print(i.shape)
            raise ValueError(f"State {state} is higher than the upper bounds {self.bounds[:,1]}.")
        res = np.empty(state.shape, dtype=int)
        np.rint((state - self.bounds[:,0]) / self.deltas, casting='unsafe', out=res) # Round to closest cell
        res = res.T # Transpose to have the n_states x n_samples shape
        res = np.squeeze(res) # Remove the extra dimension if the input was a single state
        res = tuple(res) # Convert to tuple
        return res
    
    def toState(self, index: tuple):
        """Converts an index to a state in the state space."""
        index_arr = np.array(index)
        if len(index) != self.n_states:
            raise ValueError("Index has the wrong dimension.")
        if np.any(index_arr < 0) or np.any(index_arr >= self.dims):
            raise ValueError("Index out of bounds.")
        return self.bounds[:,0] + index_arr * self.deltas
    
if __name__ == "__main__":
    bounds = np.array([[-1.0,1.0],[0.0,1.0]])
    deltas = [0.1,0.5]
    ss = DiscreteStateSpace(2,bounds,deltas)
    for i, state in ss:
        print(i, state)
    i = ss.toIndex(np.array([-1.0,0.25]))
    print(i)
    print(ss.toState(i))
