import pathlib
import numpy as np

from data_driven_legged_locomotion.common import StateSpace, MujocoEnvironment, DiscreteStateSpace

from scipy.spatial.transform import Rotation as R
import mujoco
import xml.etree.ElementTree as ET


class H1WalkEnvironment(MujocoEnvironment):
    def __init__(self, n_samples = 1000):
        base_path = pathlib.Path(__file__).parent.parent.parent.parent
        print("[DEBUG] base_path: ", base_path)
        task_candidates = [path for path in pathlib.Path(base_path).rglob("mujoco_mpc-build/mjpc/tasks/h1/walk/task.xml")]
        print("[DEBUG] task candidates: ", task_candidates)
        if len(task_candidates) == 0:
            raise ValueError(f"Could not find h1_walk task in folder {base_path}, make sure to build mujoco_mpc")
        print("Task found: ", task_candidates[0])
        model_path = task_candidates[0]
        # upper_q_bounds = np.array([5.0,5.0,1.5, # Free joint linear position
        #                             1.,1.,1.,1., # Free joint angular position
        #                             0.43, 0.43, 2.53, 2.05, 0.52, 0.43, 0.43, 2.53, 2.05, 0.52, 2.35, 2.87, 3.11, 4.45, 2.61, 2.87, 0.34, 1.3, 2.61 # Rotoidal joints angular position
        #                             ])
        # lower_q_bounds = np.array([-5.0,-5.0,0., # Free joint linear position
        #                            -1.,-1.,-1.,-1., # Free joint angular position
        #                            -0.43, -0.43, -3.14, -0.26, -0.87, -0.43, -0.43, -3.14, -0.26, -0.87, -2.35, -2.87, -0.34, -1.3,  -1.25, -2.87, -3.11, -4.45, -1.25 # Rotoidal joints angular position
        #                            ])
        upper_q_bounds = np.ones(26)*100
        lower_q_bounds = -upper_q_bounds
        upper_dq_bounds = np.ones(6+19)*350
        lower_dq_bounds = -upper_dq_bounds
        upper_bounds = np.concatenate([upper_q_bounds, upper_dq_bounds])
        lower_bounds = np.concatenate([lower_q_bounds, lower_dq_bounds])
        deltas_q = (upper_q_bounds - lower_q_bounds) / n_samples
        deltas_dq = (upper_dq_bounds - lower_dq_bounds) / n_samples
        deltas = np.concatenate([deltas_q, deltas_dq])
        ss = DiscreteStateSpace(26+25,
                        np.array(list(zip(lower_bounds, upper_bounds))),
                        deltas)
        super().__init__(ss, model_path)


    def create_obstacles(self, obstacle_positions, obstacle_sizes, obstacle_rgba=None):
        """
        Dynamically adds obstacles to the MJCF (XML) model based on positions and sizes.
        
        Args:
            xml_path (str): Path to the original XML file (model without obstacles).
            new_xml_path (str): Path to save the new XML file (model with obstacles).
            obstacle_positions (list of list): A list of [x, y, z] positions for each obstacle.
            obstacle_sizes (list of list): A list of [x_size, y_size, z_size] for each obstacle.
            obstacle_rgba (list of list): Optional list of RGBA colors for each obstacle.
                                        Default is red for all obstacles.
        """
        # Load the original XML file
        tree = ET.parse(str(self.model_path))
        root = tree.getroot()

        new_xml_path = str(self.model_path.parent / "new_model.xml")
        # Find the <worldbody> element to insert new obstacles
        worldbody = root.find('worldbody')

        # Set default RGBA colors (if not provided)
        if obstacle_rgba is None:
            obstacle_rgba = [[1, 0, 0, 1] for _ in range(len(obstacle_positions))]

        # Loop through each obstacle to add
        for i, (pos, size, rgba) in enumerate(zip(obstacle_positions, obstacle_sizes, obstacle_rgba)):
            # Create a new body for the obstacle
            obstacle_body = ET.Element('body', name=f'obstacle_{i}', pos=" ".join(map(str, pos)))

            # Create the geometry for the obstacle (we'll use a cylinder)
            geom = ET.SubElement(obstacle_body, 'geom', name=f'obstacle_geom_{i}', type="cylinder",
                                size=" ".join(map(str, size)), rgba=" ".join(map(str, rgba)))

            # Append the new obstacle to the <worldbody>
            worldbody.append(obstacle_body)

        # Save the modified XML to the new path
        tree.write(new_xml_path)

        print(f"Obstacles added and new XML saved to {new_xml_path}")

        # Update the model with the new XML
        self.model = mujoco.MjModel.from_xml_path(new_xml_path)
        self.data = mujoco.MjData(self.model)

        
    def reach_state(self, desired_state: np.ndarray, n_steps: int, viewer, video, renderer, frame_count, video_fps):
        """
        Reach a desired state in n_steps steps.
        Args:
            desired_state (np.ndarray): The desired state to reach.
            n_steps (int): The number of steps to reach the desired state.
            viewer (mujoco.MjViewer): The viewer to render the environment.
            video (mujoco.VideoRecorder): The video recorder to save the video.
            renderer (mujoco.Renderer): The renderer to render the scene.
            frame_count (int): The current frame count.
            video_fps (int): The frames per second of the video.
        Returns:
            int: The updated frame count.
        """
        # TODO: Delete this method. It is not used anymore.
        assert desired_state.shape[0] == self.n_states
        
        print("[DEBUG] n_steps: ", n_steps)
        print("[DEBUG] n_steps*self.timestep: ", n_steps*self.timestep)
        target_pos = desired_state[0:26]
        target_vel = desired_state[26:51]
             
        starting_pos = self.data.qpos.copy()
        starting_vel = self.data.qvel.copy()
        interpolator = TrajectoryInterpolator(starting_pos[7:26], starting_vel[6:25], n_steps*self.timestep, target_pos[7:26], target_vel[6:25])
        interpolator2 = TrajectoryInterpolator(starting_pos[0:3], starting_vel[0:3], n_steps*self.timestep, target_pos[0:3], target_vel[0:3])
        
        target_rotation = R.from_quat(target_pos[3:7].tolist(), scalar_first=True)
        target_angles = target_rotation.as_euler('xyz', degrees=False)

        starting_rotation = R.from_quat(starting_pos[3:7].tolist(), scalar_first=True)
        starting_angles = starting_rotation.as_euler('xyz', degrees=False)
            
        interpolator3 = TrajectoryInterpolator(starting_angles, starting_vel[3:6], n_steps*self.timestep, target_angles, target_vel[3:6])
        
        time = 0

        for i in range(n_steps):

            #
            self.data.qfrc_applied = np.zeros_like(self.data.qfrc_applied)
            self.data.xfrc_applied = np.zeros_like(self.data.xfrc_applied)
            self.data.ctrl = np.zeros_like(self.data.ctrl)
            #

            old_acc = self.data.qacc.copy()

            curr_pos = self.data.qpos.copy()
            curr_vel = self.data.qvel.copy()

            # self.data.xfrc_applied = np.zeros_like(self.data.xfrc_applied)
            # self.data.qfrc_applied = np.zeros_like(self.data.qfrc_applied)

            time += self.timestep
            self.data.qacc[6:25] = interpolator.get_acc(time)
            
            self.data.qacc[0:3] = interpolator2.get_acc(time)
            self.data.qacc[3:6] = interpolator3.get_acc(time)
            #mujoco.mj_forward(self.model, self.data)
            mujoco.mj_inverse(self.model, self.data)

            self.data.qacc = old_acc
            sol = self.data.qfrc_inverse.copy()
            
            # u = sol[6:25]
            # self.data.qfrc_applied[0:6] = sol[0:6]
            # # if an element is greater than the ctrlrange, print it
            # for i in range(len(u)):
            #     if abs(u[i]) > self.model.actuator_ctrlrange[i][1]:
            #         print(f"Control {i} is greater than ctrlrange: {u[i]}")

            # self.data.ctrl = u
            self.data.qfrc_applied = sol
            mujoco.mj_step(self.model, self.data)
        
            viewer.sync()

            if frame_count < self.data.time * video_fps:
                renderer.update_scene(self.data, camera="top")
                pixels = renderer.render()
                video.add_image(pixels)
                frame_count += 1
        
        print("Position Error: ", self.data.qpos - desired_state[:self.model.nq])
        print("Velocity Error: ", self.data.qvel - desired_state[self.model.nq:])
        
        return frame_count

class Cost:
    """Cost function for the H1 walk task"""
    def __init__(self, obstacles_positions, obstacles_sizes):
        self.obstacles_positions = obstacles_positions
        self.obstacles_sizes = obstacles_sizes
        self.alpha = 300.0
        self.beta = 1

    def get_cost_function(self) -> callable:
        """Return the cost function for the H1 walk task"""

        def cost(x: np.ndarray, k: int) -> np.ndarray:
            """
            Compute the cost for a given state x.
            Args:
                x (np.ndarray): The state to compute the cost for.
                k (int): The time step.
            Returns:
                np.ndarray: The cost for each state in x.
            
            """
            r = np.array([10.0, 10.0])
            costs = 30*np.exp(np.sqrt( ((x[0] - r[0])/4)**2 + ((x[1] - r[1])/4)**2 ))

            # z_mean = np.mean(x[2])
            # if z_mean < 0.88:
            #     costs = np.ones(x.shape[0])*1000000
            #     print("[WARNING] Torso too low. This policy could make the robot fall.")
            #     return costs

            # obstacles are modeled as bivariate gaussian obstacles
            for i in range(len(self.obstacles_positions)):
                obs = self.obstacles_positions[i]
                size = self.beta*self.obstacles_sizes[i]
                costs += self.alpha*np.exp(-((x[0] - obs[0])**2/(2*size[0]**2) + (x[1] - obs[1])**2/(2*size[1]**2)))

            return costs
            # q1 = Quaternion(x[3:7])
            # q2 = Quaternion(np.array([1.0, 0.0, 0.0, 0.0]))
            # quat_cost = 50*Quaternion.absolute_distance(q1, q2)
            # print("[DEBUG] quat_cost: ", quat_cost,"other cost: ", costs)
            # costs += quat_cost

        return cost

    # def get_cost_function(self):
    #     def cost(x, k):
    #         r = np.array([10.0, 0.0])
    #         z_torso = 0.96
    #         costs = 30*(x[:,0] - r[0])**4 + (x[:,1] - r[1])**4
    #         costs = np.squeeze(costs)
    #         costs += self.alpha*(x[:,2] - z_torso)**2
            
    #         return costs
        
    #     return cost




class TrajectoryInterpolator:
    # TODO: Delete this together with the method reach_state in H1WalkEnvironment. This is not used anymore.
    """Cubic interpolator for a trajectory in time"""
    def __init__(self, starting_qpos, starting_qvel , duration ,final_qpos, final_qvel, time=0.0):
        self.starting_qpos = starting_qpos
        self.starting_qvel = starting_qvel
        self.duration = duration
        self.final_qpos = final_qpos
        self.final_qvel = final_qvel
        self.time = time

        # Setup the interpolator by computing the coefficients of the cubic polynomial
        self.setup()

    def setup(self):
        """Setup the cubic interpolator, by computing the coefficients of the cubic polynomial"""
        self.a0 = self.starting_qpos
        self.a1 = self.starting_qvel
        self.a2 = (3*(self.final_qpos - self.starting_qpos) - (2*self.starting_qvel + self.final_qvel)*self.duration) / (self.duration**2)
        self.a3 = (2*(self.starting_qpos - self.final_qpos) + (self.starting_qvel + self.final_qvel)*self.duration) / (self.duration**3)


    def get_acc(self, t):
        """Compute the desired position at time t"""
        
        t = np.round(t,5)
        return 6*self.a3*t + 2*self.a2
