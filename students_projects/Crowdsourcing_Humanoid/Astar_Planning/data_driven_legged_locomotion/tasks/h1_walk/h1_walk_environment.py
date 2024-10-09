import pathlib
import mujoco
import numpy as np
import scipy

from data_driven_legged_locomotion.maps.h1_walk import Map
from data_driven_legged_locomotion.utils.quaternions import quat_to_forward_vector
from data_driven_legged_locomotion.common import StateSpace, MujocoEnvironment, DiscreteStateSpace

class H1WalkEnvironment(MujocoEnvironment):
    def __init__(self, ss=None, custom_model=None, starting_pos=None, starting_quat=None):
        """
        Initializes the H1WalkEnvironment class.

        Args:
            ss (StateSpace, optional): The state space. Defaults to None.
            custom_model (str, optional): The custom model path. Defaults to None.
            starting_pos (np.ndarray, optional): The starting position. Defaults to None.
        """
        if custom_model is None:
            model = H1WalkEnvironment._get_model_path()
        else:
            model = custom_model
        if ss is None:
            ss = StateSpace(26 + 25)
        super().__init__(ss, model)
        if starting_pos is not None:
            self.data.qpos[:2] = starting_pos
        if starting_quat is not None:
            self.data.qpos[3:7] = starting_quat
    
    @staticmethod
    def _get_model_path():
        """
        Finds the model path for the h1_walk task.

        Returns:
            pathlib.Path: The path to the h1_walk task model.

        Raises:
            ValueError: If the h1_walk task model cannot be found.
        """
        base_path = pathlib.Path(__file__).parent.parent.parent.parent
        print("[DEBUG] base_path: ", base_path)
        task_candidates = [path for path in pathlib.Path(base_path).rglob("mujoco_mpc-build/mjpc/tasks/h1/walk/task.xml")]
        print("[DEBUG] task candidates: ", task_candidates)
        if len(task_candidates) == 0:
            raise ValueError(f"Could not find h1_walk task in folder {base_path}, make sure to build mujoco_mpc")
        print("Task found: ", task_candidates[0])
        model_path = task_candidates[0]
        return model_path
    
    @staticmethod
    def get_fno_from_delta(qpos, qpos_old, delta_time):
        """
        Computes the forward, normal, and angular velocities from the change in position and orientation.

        Args:
            qpos (np.ndarray): The current position and orientation.
            qpos_old (np.ndarray): The previous position and orientation.
            delta_time (float): The time difference between the current and previous positions.

        Returns:
            np.ndarray: The forward, normal, and angular velocities.
        """
        vx = (qpos[0] - qpos_old[0]) / delta_time
        vy = (qpos[1] - qpos_old[1]) / delta_time
        fw = quat_to_forward_vector(qpos[3:7])
        theta = np.arctan2(fw[1], fw[0])
        v_fw = np.dot([vx, vy], np.array([np.cos(theta), np.sin(theta)]))
        v_n = np.dot([vx, vy], np.array([np.cos(theta + np.pi / 2), np.sin(theta + np.pi / 2)]))
        fw_old = quat_to_forward_vector(qpos_old[3:7])
        theta_old = np.arctan2(fw_old[1], fw_old[0])
        omega = (theta - theta_old) / delta_time
        return np.array([v_fw, v_n, omega])
        
class H1WalkEnvironmentDiscrete(H1WalkEnvironment):
    def __init__(self, n_samples=1000):
        """
        Initializes the H1WalkEnvironmentDiscrete class.

        Args:
            n_samples (int, optional): The number of samples. Defaults to 1000.
        """
        upper_q_bounds = np.ones(26) * 10
        lower_q_bounds = -upper_q_bounds
        upper_dq_bounds = np.ones(6 + 19) * 35
        lower_dq_bounds = -upper_dq_bounds
        upper_bounds = np.concatenate([upper_q_bounds, upper_dq_bounds])
        lower_bounds = np.concatenate([lower_q_bounds, lower_dq_bounds])
        deltas_q = (upper_q_bounds - lower_q_bounds) / n_samples
        deltas_dq = (upper_dq_bounds - lower_dq_bounds) / n_samples
        deltas = np.concatenate([deltas_q, deltas_dq])
        ss = DiscreteStateSpace(26 + 25,
                                np.array(list(zip(lower_bounds, upper_bounds))),
                                deltas)
        super().__init__(ss)
        
class H1WalkMapEnvironment(H1WalkEnvironment):
    def __init__(self, map: Map, starting_pos=None, starting_quat=None):
        """
        Initializes the H1WalkMapEnvironment class.

        Args:
            map (Map): The map containing obstacles.
            starting_pos (np.ndarray, optional): The starting position. Defaults to None.
        """
        self.map = map
        model_spec = mujoco.MjSpec()
        model_spec.from_file(str(H1WalkEnvironment._get_model_path()))
        map.add_to_spec(model_spec)
        model = model_spec.compile()
        self.is_triggered = False
        super().__init__(custom_model=model, starting_pos=starting_pos, starting_quat=starting_quat)
    
    def update_dynamic_obs(self):
        """
        Updates the state of dynamic obstacles in the map.
        """
        if not self.is_triggered:
            return
        self.map.step(self.model, self.timestep)
    
    def trigger(self):
        """
        Triggers the update of dynamic obstacles.
        """
        self.is_triggered = True
    
    def step(self, u: np.ndarray | float):
        """
        Advances the simulation by one step using the given action.

        Args:
            u (np.ndarray | float): The action to apply. If a float, it will be converted to a 1D array.
        """
        MujocoEnvironment.step(self, u)
        self.update_dynamic_obs()
