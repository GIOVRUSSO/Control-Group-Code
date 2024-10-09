import mujoco
import numpy as np
from pathlib import Path
from .StateSpace import StateSpace

class MujocoEnvironment:
    def __init__(self, ss: StateSpace, model_path: Path) -> None:
        self.ss = ss
        self.model_path = model_path
        model_path = str(model_path)
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.n_states = self.model.nq + self.model.nv
        self.n_actions = self.model.nu
        if self.n_states != ss.n_states:
            raise ValueError(f"State space dimension {ss.n_states} does not match model dimension {self.n_states}")
        self.data = mujoco.MjData(self.model)
    
    @property
    def timestep(self):
        return self.model.opt.timestep
    
    @property
    def time(self):
        return self.data.time
    
    def get_state(self, split=False) -> np.ndarray:
        qpos = self.data.qpos[:]
        qvel = self.data.qvel[:]
        if split:
            return qpos, qvel
        return np.concatenate((qpos, qvel))
     
    def step(self, u: np.ndarray|float):
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