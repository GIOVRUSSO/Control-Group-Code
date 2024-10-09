import mujoco
import numpy as np
from pathlib import Path
from .StateSpace import StateSpace

class MujocoEnvironment:
    def __init__(self, ss: StateSpace, model: Path | mujoco.MjModel) -> None:
        """
        Initializes the MujocoEnvironment class.

        Args:
            ss (StateSpace): The state space.
            model (Path | mujoco.MjModel): The MuJoCo model, either as a file path or an MjModel object.
        """
        self.ss = ss
        if isinstance(model, Path):
            model = str(model)
            self.model = mujoco.MjModel.from_xml_path(model)
        else:
            self.model = model
        self.n_states = self.model.nq + self.model.nv
        self.n_actions = self.model.nu
        if self.n_states != ss.n_states:
            raise ValueError(f"State space dimension {ss.n_states} does not match model dimension {self.n_states}")
        self.data = mujoco.MjData(self.model)
    
    @property
    def timestep(self):
        """
        Returns the simulation timestep.

        Returns:
            float: The simulation timestep.
        """
        return self.model.opt.timestep
    
    @property
    def time(self):
        """
        Returns the current simulation time.

        Returns:
            float: The current simulation time.
        """
        return self.data.time
    
    def get_state(self, split=False) -> np.ndarray:
        """
        Returns the current state of the simulation.

        Args:
            split (bool, optional): If True, returns qpos and qvel separately. Defaults to False.

        Returns:
            np.ndarray: The current state, concatenated if split is False, otherwise a tuple of qpos and qvel.
        """
        qpos = self.data.qpos[:]
        qvel = self.data.qvel[:]
        if split:
            return qpos, qvel
        return np.concatenate((qpos, qvel))
     
    def step(self, u: np.ndarray|float):
        """
        Advances the simulation by one step using the given action.

        Args:
            u (np.ndarray | float): The action to apply. If a float, it will be converted to a 1D array.

        Raises:
            ValueError: If the action dimension does not match the model's action dimension.
        """
        if isinstance(u, float):
            u = np.array([u])
        if u.shape != (self.n_actions,):
            raise ValueError(f"Action dimension {u.shape} does not match model dimension {self.n_actions}")
        self.data.ctrl = u
        mujoco.mj_step(self.model, self.data)
    
    def reset(self):
        mujoco.mj_reset(self.model, self.data)
        
    def launch_passive_viewer(self):
        return mujoco.viewer.launch_passive(self.model, self.data)