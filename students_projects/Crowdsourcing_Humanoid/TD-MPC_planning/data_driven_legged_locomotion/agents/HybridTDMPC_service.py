
from omegaconf import OmegaConf
from data_driven_legged_locomotion.common.StateSpace import StateSpace
from data_driven_legged_locomotion.common.ServiceSet import MujocoService
import numpy as np
import os
import pathlib
from data_driven_legged_locomotion.agents.tdmpc.tdmpc2 import TDMPC2

import torch
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import copy
import mujoco


# Setting the default configuration path
DEFAULT_CONFIG_PATH = "./config_h1/config.yaml"
 
# Setting the default directions
DEFAULT_TARGET_DIRECTION = np.array([1.0, 0.0, 0.0, 0.0]) # The neural network is trained with this reference to understand the direction of the movement.
DEFAULT_REFERENCE_DIRECTION = DEFAULT_TARGET_DIRECTION # The target direction of the movement.


class H1Controller:
    def __init__(self):
        # The action space is normalized to [-1, 1]. The following are the high and low values of the action space.
        self.action_high = np.array([0.43, 0.43, 2.53, 2.05, 0.52, 0.43, 0.43, 2.53, 2.05, 0.52, 2.35, 2.87, 3.11, 4.45, 2.61, 2.87, 0.34, 1.3, 2.61])
        self.action_low = np.array([-0.43, -0.43, -3.14, -0.26, -0.87, -0.43, -0.43, -3.14, -0.26, -0.87, -2.35, -2.87, -0.34, -1.3,  -1.25, -2.87, -3.11, -4.45, -1.25])
        # The proportional and derivative gains for the PD controller
        self.kp = np.array([50, 50, 50, 75, 10, 50, 50, 50, 75, 10, 75, 100, 100, 100, 100, 100, 100, 100, 100])
        self.kd = np.array([1.25, 1.25, 1.25, 1.5, 0.25, 1.25, 1.25, 1.25, 1.5, 0.25, 1, 2, 2, 2, 2, 2, 2, 2, 2])
        

    def unnorm_action(self,action: np.array) -> np.array:
        """Unnormalizes the action to the original action space."""
        # The action space is normalized to [-1, 1]. This function unnormalizes the action to the original action space.
        return (action + 1) / 2 * (self.action_high - self.action_low) + self.action_low

    
    def compute_joint_torques(self,data: mujoco.MjData, model: mujoco.MjModel, desired_q_pos: np.ndarray) -> np.ndarray:
        """Computes the joint torques given the desired joint positions.
            Args:
                data (mujoco.MjData): The mujoco data object.
                model (mujoco.MjModel): The mujoco model object.
                desired_q_pos (np.ndarray): The desired joint positions.
            Returns:
                np.ndarray: The joint torques.
        """
        d = data
        m = model

        # In the H1 robot model the first 7 elements of qpos are the base position and orientation. The rest of the elements are the joint positions.
        actuator_length = d.qpos[7:len(d.qpos)]
        #assert len(actuator_length) == len(desired_q_pos)
        error = desired_q_pos - actuator_length

        empty_array = np.zeros(m.actuator_dyntype.shape)

        ctrl_dot = np.zeros(m.actuator_dyntype.shape) if np.array_equal(m.actuator_dyntype,empty_array) else d.act_dot[m.actuator_actadr + m.actuator_actnum - 1]

        # In the H1 robot model the first 6 elements of qvel are the base velocity (linear + angular). The rest of the elements are the joint velocities.
        error_dot = ctrl_dot - d.qvel[6:len(d.qvel)]
        #assert len(error_dot) == len(error)

        joint_torques = self.kp*error + self.kd*error_dot

        return joint_torques

class HybridTDMPCService(MujocoService):

    def __init__(self, ss: StateSpace, model,variances: float = None, agent_horizon: int = 1, frame_skip: int = 5, delta_step: float = 0.003):
        super().__init__(ss, model, variances)
        
        base_path = pathlib.Path(__file__).parent
        print("[DEBUG] base_path: ", base_path)
        config_path_candidates = [path for path in pathlib.Path(base_path).rglob("hybrid_config.yaml")]
        agent_path_candidates = [path for path in pathlib.Path(base_path).rglob("hybrid.pt")]
        
        # Check if the config file exists
        if len(config_path_candidates) == 0:
            raise ValueError(f"Could not find agent_server binary in folder {base_path}, make sure to build the agent_server")
        
        # Check if there are multiple config files in the folder. If so, raise an error.
        if len (config_path_candidates) > 1:
            raise ValueError(f"Multiple config files found in folder {base_path}.")

        # Check if the agent file exists
        if len(agent_path_candidates) == 0:
            raise ValueError(f"Could not find agent file in folder {base_path}, make sure to build the agent_server")
        
        # Check if there are multiple agent files in the folder. If so, raise an error.
        if len (agent_path_candidates) > 1:
            raise ValueError(f"Multiple agent files found in folder {base_path}.")

        config_path = config_path_candidates[0]
        agent_path = agent_path_candidates[0]

        self.t = 0
        self.agent = None
        self.delta_step = delta_step
        self.target_reference = DEFAULT_TARGET_DIRECTION
        self.transformation_quat = None
        self.policy_reference = None
        self.agent_horizon = agent_horizon
        self.control_trajectory = []
        self.frame_skip = frame_skip

        self.set_policy_reference(DEFAULT_REFERENCE_DIRECTION)
        self._setup_agent(config_path, agent_path)

        self.controller = H1Controller()

    # def _get_joint_torques(self, action: np.array) -> np.array:
    #     """Returns the joint torques given the action."""
    #     return self.controller.compute_joint_torques(data=self.data, model=self.model, desired_q_pos=action)

    def set_data(self, data: mujoco.MjData) -> None:
        """Sets the data of the environment. The data is the mujoco data object.
            It would be ideal to copy the data object to avoid modifying the original data object or just pass the relevant information to this method. However,
            to avoid heavy computations, the data object is passed by reference. (This is a trade-off between performance and safety.).
            Args:
                data (mujoco.MjData): The mujoco data object.
        """
        # Consider refactoring this method to avoid passing the data object by reference.
        self.data = data

    def set_policy_reference(self, policy_reference: np.array) -> None:
        """Sets the policy reference. The policy reference is the desired direction of the movement for the agent.
            Args:
                policy_reference (np.array): The policy reference. It must be a quaternion.
        """
        
        # Calculate the transformation matrix from the home orientation to the target orientation
        if len(policy_reference) == 4:
            policy_reference = Quaternion(policy_reference)
        else:
            raise ValueError("The policy reference must be a quaternion.")
        
        if len(self.target_reference) == 4:
            target_quat = Quaternion(self.target_reference)
        else:
            raise ValueError("The target reference must be a quaternion.")
        self.policy_reference = policy_reference

        self.transformation_quat = target_quat * policy_reference.inverse

    # Override
    def _policy(self, x: np.ndarray, t: float = 0.0) -> np.ndarray:
        """Returns the action given the state."""
        # Instead of implementing the policy method, which returns the action given the state, the _get_next_state method is overridden to implement the policy.
        raise NotImplementedError("The policy method must be implemented.")
    
    # Override
    def _get_next_state(self, state: np.ndarray, t: float = 0.0) -> np.ndarray:
        """Returns the next state given the state and time."""
        x = copy.deepcopy(state)
        #data_copy = copy.deepcopy(self.data)
        data_copy = self.data
        agent_copy = self.agent

        # Set the base position to the origin (0,0), so to allow the agent to move in any direction without incoherence.
        x[0] = 0.0
        x[1] = 0.0

        self.control_trajectory = []
        
        for _ in range(self.agent_horizon):
            # Hybrid TD-MPC: The agent sees only a subset of x.
            x = self._generalize_walk_direction(x).numpy()
            
            x_tdmpc = self._convert_state_to_TDMPC_state(x)
            x_tdmpc = torch.tensor(x_tdmpc, dtype=torch.float32)
            action = agent_copy.act(x_tdmpc, t0=self.t == 0, task=None)
            self.t += 1
            action = action.detach().numpy()
            action = np.concatenate([action, np.zeros(8)])
            desired_joint_pos = self.controller.unnorm_action(action)
            desired_joint_pos[-8:] = np.zeros(8)
            
            for _ in range(self.frame_skip):
                u = self.controller.compute_joint_torques(data=data_copy, model=self.model, desired_q_pos=desired_joint_pos)
                self.control_trajectory.append(u.copy())
                data_copy.ctrl = u.copy()
                mujoco.mj_step(self.model, data_copy)

            x = np.concatenate([data_copy.qpos, data_copy.qvel])
            x[0] = 0.0
            x[1] = 0.0

        next_state = np.concatenate([data_copy.qpos, data_copy.qvel])
        # self._last_u = u
        self.data.qpos = copy.deepcopy(state[0:self.model.nq])
        self.data.qvel = copy.deepcopy(state[self.model.nq:])
        self.data.time = t
        self.data.ctrl = u.copy()
        #return self.control_trajectory
        return next_state

    def _get_next_state_simple_model(self, state: np.ndarray, t: float = 0.0) -> np.ndarray:
        # Make a copy of the state to avoid modifying the original state
        state = copy.deepcopy(state)

        transformation_rot = R.from_quat(self.transformation_quat.inverse.q, scalar_first=True) # Inverse of the transformation quaternion. Transformation quaternion is from the reference orientation to the target orientation, that corresponds to negative rotation.
        angle = transformation_rot.as_euler('zyx')[0]
        angle = angle % (2*np.pi) # Normalize the angle to the range [0, 2*pi]

        # Update the position of the robot according to the direction of the movement
        state[0] += self.delta_step*np.cos(angle)
        state[1] += self.delta_step*np.sin(angle)

        #print("[DEBUG] angle", angle,"increment (dx,dy):", self.delta_step*np.cos(angle), self.delta_step*np.sin(angle))
        return state 
    

    def get_control(self, state: np.ndarray, t: float = 0.0) -> np.ndarray:
        """Returns the control input given the state and time."""

        x = copy.deepcopy(state)
        # Set the base position to the origin (0,0), so to allow the agent to move in any direction without incoherence.
        x[0] = 0.0
        x[1] = 0.0

        x = self._generalize_walk_direction(x).numpy()
            
        x_tdmpc = self._convert_state_to_TDMPC_state(x)
        x_tdmpc = torch.tensor(x_tdmpc, dtype=torch.float32)
        action = self.agent.act(x_tdmpc, t0=self.t == 0, task=None)
        self.t += 1
        action = action.detach().numpy()
        action = np.concatenate([action, np.zeros(8)])

        desired_joint_pos = self.controller.unnorm_action(action)
        # Make sure the last 8 elements are zeros
        desired_joint_pos[-8:] = np.zeros(8)
        # Compute the joint torques from the desired joint positions
        u = self.controller.compute_joint_torques(data=self.data, model=self.model, desired_q_pos=desired_joint_pos)
        return u

    def _convert_state_to_TDMPC_state(self, x: np.ndarray) -> np.ndarray:
        """ Converts the state to the state that the TD-MPC2 agent can understand.
            In particular, in the hybrid architecture, the agent sees only a subset of the state:
            - The first 18 elements of the state: which correspond to the base position and orientation, and the joint positions of the lower body (torso included).
            - The elements from 26 to 43: which correspond to the base velocity (linear + angular), and the joint velocities of the lower body (torso included).
            Args:
                x (np.ndarray): The state of the environment.
            Returns:
                np.ndarray: The state that the TD-MPC2 agent can understand.
        """
        x_tdmpc = np.concatenate([x[0:18], x[26:43]]) # x_tdmpc = [x[0:18], x[26:43]] [x[0:26-8], x[26:51-8]]
        #print("x_tdmpc: ", x_tdmpc, "len(x_tdmpc): ", len(x_tdmpc))
        return x_tdmpc
    
    def _setup_agent(self,config_path: str, agent_path: str)-> None:
        """Utility method to setup the agent. It loads the configuration file and the agent file.
            Args:
                config_path (str): The path to the configuration file.
                agent_path (str): The path to the agent file.
        """

        # check if config path exists
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config path {config_path} does not exist.")

        # check if agent path exists
        if not os.path.exists(agent_path):
            raise FileNotFoundError(f"Agent path {agent_path} does not exist.")

        # Load the configuration file
        cfg = OmegaConf.load(config_path)

        # Create the TD-MPC agent
        self.agent = TDMPC2(cfg)        
        
        # Load agent
        self.agent.load(agent_path)
        print("Loaded agent from: ", agent_path)
        
    def _generalize_walk_direction(self,obs: np.ndarray)-> np.ndarray:
        """This private method generalizes the direction of the walk. It rotates the state by the transformation quaternion.
            So the agent can move in any direction. The transformation quaternion is calculated as the product of the target quaternion and the policy quaternion.
            Args:
                obs (np.ndarray): The state of the environment.
            Returns:
                np.ndarray: The "generalized" state of the environment. That is, the state of the environment after the rotation by the transformation quaternion.
                
        """        

        transformation_quat = self.transformation_quat

        current_quat = Quaternion(obs[3:7])  # Convert tensor slice to numpy array for Quaternion
        current_position = obs[0:3] # Convert tensor slice to numpy array for Quaternion

        new_quat = (transformation_quat * current_quat).elements.astype(float)
        new_pos = transformation_quat.rotate(current_position).astype(float)
        new_vel = transformation_quat.rotate(obs[26:29]).astype(float)
        
        # convert obs to tensor
        obs = torch.from_numpy(obs).type(torch.FloatTensor)
        #obs = torch.tensor(obs, dtype=torch.float32)

        obs[0:3] = torch.from_numpy(new_pos).type(torch.FloatTensor)
        obs[3:7] = torch.from_numpy(new_quat).type(torch.FloatTensor)
        obs[26:29] = torch.from_numpy(new_vel).type(torch.FloatTensor)

        return obs
    
    def get_agent_copy(self) -> TDMPC2:
        """Returns a copy of the agent.
            Returns:
                TDMPC2: A copy of the agent.
        """
        return self.agent.copy()

    def set_agent_copy(self, agent_copy: TDMPC2):
        """Sets the agent copy.
        
            Args:
                agent_copy (TDMPC2): The agent copy.
            """
        self.agent = agent_copy
