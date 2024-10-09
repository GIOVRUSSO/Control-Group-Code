import mujoco
from mujoco_mpc import agent as agent_lib
import numpy as np
import pathlib

from data_driven_legged_locomotion.common import MujocoService, StateSpace, OfflineReaderService, Behavior, NormalStateCondPF, SingleBehavior
from data_driven_legged_locomotion.utils.quaternions import quat_to_forward_vector, rotate_quat
from data_driven_legged_locomotion.tasks.h1_walk.h1_walk_environment import H1WalkEnvironment

class H1WalkAgent():
    """An agent based on Mujoco MPC that is capable of walking to a target pose."""
    def __init__(self, env):
        model_path = str(H1WalkEnvironment._get_model_path())
        model = mujoco.MjModel.from_xml_path(model_path)
        # Find the agent_server binary
        base_path = pathlib.Path(__file__).parent.parent.parent.parent.parent
        print("[DEBUG] base_path: ", base_path)
        server_binary_candidates = [path for path in pathlib.Path(base_path).rglob("agent_server")]
        print("[DEBUG] server_binary_candidates: ", server_binary_candidates)
        if len(server_binary_candidates) == 0:
            raise ValueError(f"Could not find agent_server binary in folder {base_path}, make sure to build the agent_server")
        print("[DEBUG] Agent server binary found: ", server_binary_candidates[0])
        server_binary_path = server_binary_candidates[0]
        # Create and configure the agent
        agent = agent_lib.Agent(task_id="H1 Walk", 
                                model=model, 
                                server_binary_path=server_binary_path)
        agent.set_cost_weights({'Posture arms': 0.06, 'Posture torso': 0.05, 'Face goal': 4.0})
        
        # Initialize the attributes
        self.agent = agent
        self.data = mujoco.MjData(model)
        self.env = env
        
        # Update the mocap position
        self._sync_env_state()
        
    def set_reference(self, pos, quat):
        self.data.mocap_pos = pos
        self.data.mocap_quat = quat
    
    def _sync_env_state(self):
        self.data.time = self.env.data.time
        self.data.qpos = self.env.data.qpos
        self.data.qvel = self.env.data.qvel
        self.data.act = self.env.data.act
        self.data.userdata = self.env.data.userdata
    
    def get_control_action(self) -> np.ndarray:
        self._sync_env_state()
        self.agent.set_state(
            time=self.data.time,
            qpos=self.data.qpos,
            qvel=self.data.qvel,
            act=self.data.act,
            mocap_pos=self.data.mocap_pos,
            mocap_quat=self.data.mocap_quat,
            userdata=self.data.userdata,
        )
        self.agent.planner_step()
        u = self.agent.get_action(nominal_action=False)
        return u
    
class H1PolarWalkAgent(H1WalkAgent):
    def set_command(self, command):
        if command not in ["FORWARD", "LEFT", "RIGHT"]:
            raise ValueError(f"Command {command} is not valid")
        self._sync_env_state()
        fw = quat_to_forward_vector(self.data.qpos[3:7])
        pos_ref = self.data.qpos[:3]
        quat_ref = self.data.qpos[3:7]
        if command == "FORWARD":
            pos_ref = pos_ref + np.concatenate([fw, [0]])
        elif command == "LEFT":
            fw = quat_to_forward_vector(rotate_quat(self.data.qpos[3:7], np.pi/4))
            pos_ref = pos_ref + np.concatenate([fw, [0]])
        elif command == "RIGHT":
            fw = quat_to_forward_vector(rotate_quat(self.data.qpos[3:7], -np.pi/4))
            pos_ref = pos_ref + np.concatenate([fw, [0]])
        self.set_reference(pos_ref, quat_ref)