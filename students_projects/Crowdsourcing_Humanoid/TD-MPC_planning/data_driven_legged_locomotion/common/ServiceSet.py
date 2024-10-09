from abc import ABC, abstractmethod
import mujoco
import numpy as np

from .StatePF import StateCondPF, NormalStatePF, FakeStateCondPF
from .StateSpace import StateSpace


class Behavior(ABC): #{pi(x_k|x_k-1)}_0:N
    """A Behavior is a sequence of InstantBehaviors that describes the behavior of a system in a finite span of time."""
    def __init__(self, ss: StateSpace, time_window: int):
        self.ss = ss
        self.N = time_window
        self.time_window: list[StateCondPF] = []
    
    def getAtTime(self, k: int) -> StateCondPF:
        """Returns the state conditional probability distribution at time k."""
        if k < 0 or k > self.N - 1:
            raise ValueError(f"Time index k must be between 0 and {self.N-1}.")
        return self.time_window[k]

class Service(ABC):
    """A Service provides a behavior to the crowdsourcing algorithm."""
    def __init__(self, ss: StateSpace):
        self.ss = ss
    
    @property
    def last_u(self) -> np.ndarray:
        raise ValueError("The service does not have a control action.")
    
    @property
    def last_t(self) -> float:
        raise ValueError("The service does not have a control action.")
    
    @abstractmethod
    def _generateBehavior(self, state: np.ndarray, N: int, t: float = 0.0) -> Behavior:
        """Generates a behavior for the given state."""
        pass
    
    def generateBehavior(self, state: np.ndarray, N: int, t: float = 0.0) -> Behavior:
        """Generates a behavior for the given state."""
        return self._generateBehavior(state, N, t)

class BehaviorSet: #{{pi(x_k|x_k-1)}_0:N}_1:S
    """A set of behaviors that provide a set of state conditional probability distributions to the crowdsourcing algorithm."""
    def __init__(self, ss: StateSpace, N: int):
        self.ss = ss
        self.behaviors: list[Behavior] = []
        self.N = N
    
    def __len__(self):
        return len(self.behaviors)
    
    @property
    def S(self):
        return len(self.behaviors)
    
    def add(self, behavior: Behavior):
        """Adds a behavior to the set."""
        if behavior.ss != self.ss:
            raise ValueError("Behavior state space does not match the state space of the BehaviorSet.")
        if behavior.N != self.N:
            raise ValueError("Behavior time window does not match the time window of the BehaviorSet.")
        self.behaviors.append(behavior)
        
    def getAtTime(self, k: int) -> list[StateCondPF]:
        """Returns the state conditional probability distributions at time k given by all behaviors."""
        return [behavior.getAtTime(k) for behavior in self.behaviors]
    
    def extractBehavior(self, s_list: list[int]) -> Behavior:
        """Extracts a behavior from a list of service indices."""
        if len(s_list) != self.N:
            raise ValueError("The list of indices must have the same length as the time window.")
        behavior = Behavior(self.ss, self.N)
        for k, s in enumerate(s_list):
            behavior.time_window.append(self.behaviors[s].time_window[k])
        return behavior

class ServiceSet:
    """A set of services that provide a set of behaviors to the crowdsourcing algorithm."""
    def __init__(self, ss: StateSpace):
        self.ss = ss
        self.services: list[Service] = []
    
    def __len__(self):
        return len(self.services)
    
    def addService(self, service: Service):
        """Adds a service to the set."""
        if service.ss != self.ss:
            raise ValueError("Service state space does not match the state space of the ServiceSet.")
        self.services.append(service)
    
    def getNumServices(self) -> int:
        """Returns the number of services."""
        return len(self.services)
    
    def getBehaviors(self, x_0: np.ndarray, N: int, time: float = 0.0) -> BehaviorSet:
        behavior_set = BehaviorSet(self.ss, N)
        for service in self.services:
            behavior_set.add(service.generateBehavior(x_0, N, time))
        return behavior_set
    
class SingleBehavior(Behavior):
    def __init__(self, ss: StateSpace, behavior: StateCondPF):
        super().__init__(ss, 1)
        self.time_window.append(behavior)
        
class MujocoService(Service):
    """A service that exploits a deterministic policy in a Mujoco environment to generate behaviors.
       The policy is rolled out in the environment to generate the next state, then a PF is created
       adding Gaussian noise to the next state."""
    def __init__(self, ss: StateSpace, model, variances: float = None, policy_sampling_time: float = 0.02, enable_zoh: bool = True):
        super().__init__(ss)
        if variances is None:
            variances = np.ones(ss.n_states) * 0.01
        self.variances = variances
        self.model = model
        self.data = mujoco.MjData(model)
        
        # Zero-order hold attributes
        self._is_zoh_enabled = enable_zoh
        self._policy_sampling_time = policy_sampling_time
        self._last_u = np.zeros(model.nu) # Last control action
        self._last_t = 0.0 # Last time the policy was evaluated
        
        # Check model consistency
        if np.any(variances <= 0):
            raise ValueError("Variance must be positive.")
        model_states = model.nq + model.nv
        if model_states != self.ss.n_states:
            raise ValueError(f"State space dimensions {self.ss.n_states} do not match the Mujoco model {model_states}.")
    
    @property
    def last_u(self):
        return self._last_u
    
    @property
    def last_t(self):
        return self._last_t
    
    def policy(self, x: np.ndarray, t: float = 0.0) -> np.ndarray:
        """Returns the control action for the given state at the given time. If enable_zoh is True,
        this method takes the policy sampling frequency into account by applying a zero-order hold to the policy.
        Otherwise, the policy is always evaluated."""
        if not self._is_zoh_enabled:
            self._last_u = self._policy(x,t)
            self._last_t = t
            print(f"[DEBUG] MujocoService {id(self)} policy evaluated at time {t}.")
            return self._last_u
        elapsed_time = t - self._last_t
        if elapsed_time >= self._policy_sampling_time or t == 0.0: # We always evaluate the policy at t=0
            self._last_t = t
            self._last_u = self._policy(x,t)
            print(f"[DEBUG] MujocoService {id(self)} policy evaluated at time {t}.")
        return self._last_u
    
    @abstractmethod
    def _policy(self, x: np.ndarray, t: float = 0.0) -> np.ndarray:
        """Returns the control action for the given state. This method does not take
        the policy sampling frequency into account, thus it should be used only by the
        MujocoService class."""
        pass
    
    def _get_next_state(self, state: np.ndarray, t: float = 0.0) -> np.ndarray:
        """Returns the next state given the current state and the current time. This method must set _last_u
        as the control action used to reach the next state."""
        u = self.policy(state,t)
        self._last_u = u
        self.data.qpos = state[0:self.model.nq]
        self.data.qvel = state[self.model.nq:]
        self.data.time = t
        self.data.ctrl = u
        mujoco.mj_step(self.model, self.data)
        return np.concatenate([self.data.qpos, self.data.qvel])
    
    def _generateBehavior(self, state: np.ndarray, N: int, t: float = 0.0) -> Behavior:
        """Generates a behavior for the given state."""
        if N > 1:
            raise ValueError("MujocoService only supports N=1.")
        x_next = self._get_next_state(state, t)
        pf = NormalStatePF(self.ss, x_next, np.diag(self.variances))
        cond_pf = FakeStateCondPF(self.ss, pf)
        return SingleBehavior(self.ss, cond_pf)
    
class OfflineReaderService(Service):
    """A service that reads a behavior from a file."""
    def __init__(self, ss: StateSpace, file_path: str):
        super().__init__(ss)
        self.file_path = file_path
        self.behavior = self._readBehavior(file_path)
    
    def _readBehavior(self, file_path: str) -> Behavior:
        """Reads a behavior from a file."""
        raise NotImplementedError("OfflineReaderService._readBehavior must be implemented.")
    
    def _generateBehavior(self, state: np.ndarray, N: int, t: float = 0.0) -> Behavior:
        """Generates a behavior for the given state."""
        return self.behavior