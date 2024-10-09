import control
import numpy as np
from data_driven_legged_locomotion.common import MujocoService, StateSpace

class LQRService(MujocoService):
  def __init__(self, ss: StateSpace, model, variances: float = None):
    super().__init__(ss, model, variances)
    g = 9.81
    m = 5.5
    l = 0.5
    b = 3.5
    A = [[0, 1], [g/l, -b/(m*l**2)]]
    B = [[0], [1/(m*l**2)]]
    C = [[1, 0],[0, 1]]
    D = 0
    sys = control.ss(A, B, C, D)
    sys = control.c2d(sys, 0.002)
    Q = [[1, 0], [0, 1]]
    R = [0.1]
    self.K, S, E = control.dlqr(sys, Q, R)
  
  def _policy(self, x: np.ndarray) -> np.ndarray:
    (q, dot_q) = x
    delta_q = q - np.pi
    delta_dot_q = dot_q
    delta_x = np.array([delta_q, delta_dot_q])
    u = - np.dot(self.K, delta_x)
    u = np.reshape(u, (1,))
    return u