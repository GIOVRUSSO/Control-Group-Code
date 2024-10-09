import numpy as np
from data_driven_legged_locomotion.common import MujocoService, StateSpace

class SwingUpService(MujocoService):
    """This service allows the pendulum to swing up. It is based on the energy shaping control method."""
    def __init__(self, ss: StateSpace, model, variances: float = None):
        super().__init__(ss, model, variances)
        self.g = 9.81
        self.m = 5.5
        self.l = 0.5
        self.b = 3.5
        self.k = 10
    
    def _policy(self, x: np.ndarray) -> np.ndarray:
        (q, dot_q) = x
        E_actual = 0.5 * self.m * self.l**2 * dot_q**2 - self.m * self.g * self.l * np.cos(q)
        E_desired = self.m * self.g * self.l
        tilde_E = E_actual - E_desired
        u = - self.k * dot_q * tilde_E
        return u