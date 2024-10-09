from abc import ABC, abstractmethod
import functools
import mujoco
import numpy as np
import pathlib
import scipy

from data_driven_legged_locomotion.utils.mujoco_spec import attach_spec
from data_driven_legged_locomotion.utils.quaternions import quat_to_forward_vector

COST_HIGH = 100.0

class Obstacle(ABC):
    def __init__(self):
        """
        Initializes the Obstacle class.
        """
        pass
    
    @abstractmethod
    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        raise NotImplementedError("Method not implemented.")
    
    @abstractmethod
    def add_to_spec(self, model_spec: dict):
        """
        Adds the obstacle to the model specification.

        Args:
            model_spec (dict): The model specification to which the obstacle is added.
        """
        raise NotImplementedError("Method not implemented.")
    
class DynamicObstacle(Obstacle):
    def __init__(self):
        """
        Initializes the DynamicObstacle class.
        """
        self.transition_end = False
    
    @abstractmethod
    def step(self, delta_t: float):
        """
        Advances the obstacle's state by a given time step.

        Args:
            delta_t (float): The time step by which to advance the obstacle's state.
        """
        raise NotImplementedError("Method not implemented.")

class MeshObstacle(Obstacle):
    last_id = 0
    
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
        super().__init__()
        self.xml_path = None
        self.resource_dir = None
        self.pos = pos
        self.quat = quat
        self.yaw = scipy.spatial.transform.Rotation.from_quat(quat,scalar_first=True).as_euler('zyx')[0]
        self.name = 'mesh_obstacle'
        self.id = MeshObstacle.last_id
        MeshObstacle.last_id += 1
    
    def add_to_spec(self, model_spec: dict):
        """
        Adds the cylinder to the model specification.

        Args:
            model_spec (dict): The model specification to which the cylinder is added.
        """
        spec = mujoco.MjSpec()
        spec.from_file(str(self.xml_path))
        attach_spec(model_spec, spec, self.resource_dir, prefix=f"{self.name}_{self.id}_", body_pos=np.append(self.pos, [0.0]), body_quat=self.quat)

class DynamicMeshObstacle(MeshObstacle, DynamicObstacle):
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
        MeshObstacle.__init__(self, pos, quat)
        DynamicObstacle.__init__(self)
        

class Wall(Obstacle):
    last_id = 0
    
    def __init__(self, start_pos: np.ndarray, end_pos: np.ndarray, width: float = 0.07, height: float = 1.0):
        """
        Initializes the Wall class.

        Args:
            start_pos (np.ndarray): The starting position of the wall.
            end_pos (np.ndarray): The ending position of the wall.
            width (float, optional): The width of the wall. Defaults to 0.2.
            height (float, optional): The height of the wall. Defaults to 1.0.
        """
        super().__init__()
        self.id = SlidingWall.last_id
        SlidingWall.last_id += 1
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.width = width
        self.height = height
        self.length = np.linalg.norm(end_pos - start_pos)
        self.yaw = np.arctan2(end_pos[1] - start_pos[1], end_pos[0] - start_pos[0])
        self.mean_point = (start_pos + end_pos) / 2
        self.pos = np.zeros(2)  # Position of the center of the obstacle
        self.versor = (self.end_pos - self.start_pos) / self.length
        self.normal = np.array([-self.versor[1], self.versor[0]])
        
        
    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        bump_radius = 0.5 + self.width / 2
        dist_on_line = np.dot(pos - self.start_pos, self.versor)
        dist_from_line = np.abs(np.dot(pos - self.start_pos, self.normal))
        if dist_on_line > 0 and dist_on_line < self.length and dist_from_line < self.width / 2:
            return COST_HIGH
        if dist_on_line < 0:
            distance_from_segment = np.linalg.norm(pos - self.start_pos)
        elif dist_on_line > self.length:
            distance_from_segment = np.linalg.norm(pos - self.end_pos)
        else:
            distance_from_segment = dist_from_line
        if distance_from_segment < bump_radius:
            return COST_HIGH * (1.0 - distance_from_segment/bump_radius)
        return 0.0
        
    
    def add_to_spec(self, model_spec: dict):
        """
        Adds the sliding wall to the model specification.

        Args:
            model_spec (dict): The model specification to which the sliding wall is added.
        """
        body = model_spec.worldbody.add_body()
        body.name = f"obstacle_wall_{self.id}"
        body.pos = self.mean_point.tolist() + [self.height]
        body.quat = scipy.spatial.transform.Rotation.from_euler('z', self.yaw).as_quat(scalar_first=True)
        geom = body.add_geom()
        geom.name = f"obstacle_wall_geom_{self.id}"
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = [self.length / 2, self.width, self.height]
        geom.rgba = [197.0/255, 194.0/255, 199.0/255, 1]

class Cylinder(Obstacle):
    last_id = 0
    
    def __init__(self, pos: np.ndarray, radius: float = 0.8, height: float = 0.5):
        """
        Initializes the Cylinder class.

        Args:
            pos (np.ndarray): The position of the cylinder.
            radius (float, optional): The radius of the cylinder. Defaults to 0.8.
            height (float, optional): The height of the cylinder. Defaults to 0.5.
        """
        super().__init__()
        self.id = Cylinder.last_id
        Cylinder.last_id += 1
        self.radius = radius
        self.height = height
        self.pos = pos

    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        return 30 * scipy.stats.multivariate_normal.pdf(pos, mean=self.pos, cov=0.4 * np.eye(2))
    
    def add_to_spec(self, model_spec: dict):
        """
        Adds the cylinder to the model specification.

        Args:
            model_spec (dict): The model specification to which the cylinder is added.
        """
        body = model_spec.worldbody.add_body()
        body.name = f"obstacle_{self.id}"
        body.pos = self.pos.tolist() + [self.height]
        geom = body.add_geom()
        geom.name = f"obstacle_geom_{self.id}"
        geom.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        geom.size = [self.radius, self.radius, self.height]
        geom.rgba = [1, 0, 0, 1]
        
class Table(MeshObstacle):
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
        """
        Initializes the Table class.

        Args:
            pos (np.ndarray): The position of the table.
        """
        super().__init__(pos, quat)
        self.xml_path = pathlib.Path(__file__).parent / "obstacles" / "table" / "table.xml"
        self.resource_dir = self.xml_path.parent
        self.name = 'table'
        self.radius = 1.0

    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        bump_radius = 0.5
        r = np.linalg.norm(pos - self.pos)
        if r < self.radius:
            return COST_HIGH
        elif r < self.radius + bump_radius:
            return COST_HIGH * (1.0 - (r - self.radius) / bump_radius)
        return 0.0

class Lamp(MeshObstacle):
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
        """
        Initializes the Lamp class.

        Args:
            pos (np.ndarray): The position of the lamp.
        """
        super().__init__(pos, quat)
        self.xml_path = pathlib.Path(__file__).parent / "obstacles" / "lamp" / "lamp.xml"
        self.resource_dir = self.xml_path.parent
        self.name = 'lamp'
        self.radius = 0.5

    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        bump_radius = 0.5
        r = np.linalg.norm(pos - self.pos)
        if r < self.radius:
            return COST_HIGH
        elif r < self.radius + bump_radius:
            return COST_HIGH * (1.0 - (r - self.radius) / bump_radius)
        return 0.0
    
class Pouf(MeshObstacle):
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
        """
        Initializes the Lamp class.

        Args:
            pos (np.ndarray): The position of the lamp.
        """
        super().__init__(pos, quat)
        self.xml_path = pathlib.Path(__file__).parent / "obstacles" / "pouf" / "pouf.xml"
        self.resource_dir = self.xml_path.parent
        self.name = 'pouf'
        self.radius = 0.5

    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        bump_radius = 0.5
        r = np.linalg.norm(pos - self.pos)
        if r < self.radius:
            return COST_HIGH
        elif r < self.radius + bump_radius:
            return COST_HIGH * (1.0 - (r - self.radius) / bump_radius)
        return 0.0

class Shelf(MeshObstacle):
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
        """
        Initializes the Shelf class.

        Args:
            pos (np.ndarray): The position of the shelf.
        """
        super().__init__(pos, quat=quat)
        self.xml_path = pathlib.Path(__file__).parent / "obstacles" / "shelf" / "shelf.xml"
        self.resource_dir = self.xml_path.parent
        self.name = 'shelf'
        self.width = 0.61
        self.length = 1.22

    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        bump_radius = 0.5 + self.width / 2
        current_yaw = self.yaw
        versor = np.array([np.cos(current_yaw), np.sin(current_yaw)])
        normal = -np.array([-versor[1], versor[0]])
        dist_on_line = np.abs(np.dot(pos - self.pos, versor))
        dist_from_line = np.dot(pos - self.pos, normal)
        center = self.pos + self.width / 2 * normal
        if dist_on_line < self.length / 2 and dist_from_line > 0 and dist_from_line < self.width:
            return COST_HIGH
        if dist_on_line > self.length / 2:
            distance_from_segment = min(np.linalg.norm(pos - center - self.length / 2 * versor), np.linalg.norm(pos - center + self.length / 2 * versor))
        else:
            distance_from_segment = np.abs(np.dot(pos - center, normal))
        if distance_from_segment < bump_radius:
            return COST_HIGH * (1.0 - distance_from_segment/bump_radius)
        return 0.0
    
class TV(MeshObstacle):
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
        """
        Initializes the TV class.

        Args:
            pos (np.ndarray): The position of the tv.
        """
        super().__init__(pos, quat=quat)
        self.xml_path = pathlib.Path(__file__).parent / "obstacles" / "tv" / "tv.xml"
        self.resource_dir = self.xml_path.parent
        self.name = 'tv'
        self.width = 0.395
        self.length = 1.47

    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        bump_radius = 0.5 + self.width / 2
        current_yaw = self.yaw
        versor = np.array([np.cos(current_yaw), np.sin(current_yaw)])
        normal = np.array([-versor[1], versor[0]])
        dist_on_line = np.abs(np.dot(pos - self.pos, versor))
        dist_from_line = np.dot(pos - self.pos, normal)
        center = self.pos + self.width / 2 * normal
        if dist_on_line < self.length / 2 and dist_from_line > 0 and dist_from_line < self.width:
            return COST_HIGH
        if dist_on_line > self.length / 2:
            distance_from_segment = min(np.linalg.norm(pos - center - self.length / 2 * versor), np.linalg.norm(pos - center + self.length / 2 * versor))
        else:
            distance_from_segment = np.abs(np.dot(pos - center, normal))
        if distance_from_segment < bump_radius:
            return COST_HIGH * (1.0 - distance_from_segment/bump_radius)
        return 0.0
    
class Bookshelf(MeshObstacle):
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
        """
        Initializes the Bookshelf class.

        Args:
            pos (np.ndarray): The position of the tv.
        """
        super().__init__(pos, quat=quat)
        self.xml_path = pathlib.Path(__file__).parent / "obstacles" / "bookshelf" / "bookshelf.xml"
        self.resource_dir = self.xml_path.parent
        self.name = 'bookshelf'
        self.width = 0.299
        self.length = 2.41

    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        bump_radius = 0.55 + self.width
        current_yaw = self.yaw
        versor = np.array([np.cos(current_yaw), np.sin(current_yaw)])
        normal = np.array([-versor[1], versor[0]])
        dist_on_line = np.abs(np.dot(pos - self.pos, versor))
        dist_from_line = np.dot(pos - self.pos, normal)
        center = self.pos + self.width / 2 * normal
        if dist_on_line < self.length / 2 and dist_from_line > 0 and dist_from_line < self.width:
            return COST_HIGH
        if dist_on_line > self.length / 2:
            distance_from_segment = min(np.linalg.norm(pos - center - self.length / 2 * versor), np.linalg.norm(pos - center + self.length / 2 * versor))
        else:
            distance_from_segment = np.abs(np.dot(pos - center, normal))
        if distance_from_segment < bump_radius:
            return COST_HIGH * (1.0 - distance_from_segment/bump_radius)
        return 0.0
    
class Couch(MeshObstacle):
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])):
        """
        Initializes the Couch class.

        Args:
            pos (np.ndarray): The position of the couch.
        """
        super().__init__(pos, quat=quat)
        self.xml_path = pathlib.Path(__file__).parent / "obstacles" / "couch" / "couch.xml"
        self.resource_dir = self.xml_path.parent
        self.name = 'couch'
        self.width = 0.866
        self.length = 1.92

    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        bump_radius = 0.52 + self.width / 2
        current_yaw = self.yaw
        versor = np.array([np.cos(current_yaw), np.sin(current_yaw)])
        normal = np.array([-versor[1], versor[0]])
        dist_on_line = np.abs(np.dot(pos - self.pos, versor))
        dist_from_line = np.abs(np.dot(pos - self.pos, normal))
        if dist_on_line < self.length / 2 and dist_from_line < self.width / 2:
            return COST_HIGH
        if dist_on_line > self.length / 2:
            distance_from_segment = min(np.linalg.norm(pos - self.pos - self.length / 2 * versor), np.linalg.norm(pos - self.pos + self.length / 2 * versor))
        else:
            distance_from_segment = dist_from_line
        if distance_from_segment < bump_radius:
            return COST_HIGH * (1.0 - distance_from_segment/bump_radius)
        return 0.0

class Door(DynamicMeshObstacle):
    def __init__(self, pos: np.ndarray, quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0]), shift_yaw: float = 0.0):
        """
        Initializes the Door class.

        Args:
            pos (np.ndarray): The position of the shelf.
        """
        super().__init__(pos, quat)
        self.xml_path = pathlib.Path(__file__).parent / "obstacles" / "door" / "door.xml"
        self.resource_dir = self.xml_path.parent
        self.name = 'door'
        self.shift_yaw = shift_yaw # The amount by which the door is rotated after a transition
        self.rotated_yaw = 0.0 # The current rotation of the door with respect to the initial position
        self.length = 1.0
        self.width = 0.11

    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        bump_radius = 0.10
        current_yaw = self.yaw + self.rotated_yaw + np.pi # Accounts for the mesh orientation
        versor = np.array([np.cos(current_yaw), np.sin(current_yaw)])
        normal = np.array([-versor[1], versor[0]])
        dist_on_line = np.dot(pos - self.pos, versor)
        dist_from_line = np.abs(np.dot(pos - self.pos, normal))
        if dist_on_line < 0 - bump_radius or dist_on_line > self.length + bump_radius:
            return 0.0
        if dist_from_line > self.width / 2 + bump_radius:
            return 0.0
        return COST_HIGH
    
    def step(self, model: mujoco.MjModel, delta_t: float):
        """
        Advances the door's state by a given time step.

        Args:
            model (mujoco.MjModel): The MuJoCo model.
            delta_t (float): The time step by which to advance the door's state.
        """
        if self.transition_end:
            return
        if np.abs(self.rotated_yaw) > np.abs(self.shift_yaw):
            self.transition_end = True
            return
        direction = np.sign(self.shift_yaw)
        self.rotated_yaw += direction * delta_t * 1.5
        body = model.body(f"{self.name}_{self.id}_door")
        body.quat = scipy.spatial.transform.Rotation.from_euler('z', self.yaw + self.rotated_yaw).as_quat(scalar_first=True)
    

class SlidingWall(DynamicObstacle):
    last_id = 0
    
    def __init__(self, start_pos: np.ndarray, end_pos: np.ndarray, shift_vect: np.ndarray, width: float = 0.2, height: float = 1.0):
        """
        Initializes the SlidingWall class.

        Args:
            start_pos (np.ndarray): The starting position of the wall.
            end_pos (np.ndarray): The ending position of the wall.
            shift_vect (np.ndarray): The shift vector for the wall.
            width (float, optional): The width of the wall. Defaults to 0.2.
            height (float, optional): The height of the wall. Defaults to 1.0.
        """
        super().__init__()
        self.id = SlidingWall.last_id
        SlidingWall.last_id += 1
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.shift_vect = shift_vect
        self.width = width
        self.height = height
        self.length = np.linalg.norm(end_pos - start_pos)
        self.yaw = np.arctan2(end_pos[1] - start_pos[1], end_pos[0] - start_pos[0])
        self.mean_point = (start_pos + end_pos) / 2
        self.pos = np.zeros(2)  # Position of the center of the obstacle
        
    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the cost associated with the given position.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The computed cost.
        """
        n_obs = max(1, int(self.length / 1.0))
        obs_perc = np.linspace(0, 1, n_obs)
        start_point, end_point = self.start_pos, self.end_pos
        if self.transition_end:
            start_point = start_point + self.shift_vect
            end_point = end_point + self.shift_vect
        c = 0.0
        for perc in obs_perc:
            obs_point = start_point + perc * (end_point - start_point)
            c += 100 * scipy.stats.multivariate_normal.pdf(pos, mean=obs_point, cov=0.2 * np.eye(2))
        return c
    
    def add_to_spec(self, model_spec: dict):
        """
        Adds the sliding wall to the model specification.

        Args:
            model_spec (dict): The model specification to which the sliding wall is added.
        """
        body = model_spec.worldbody.add_body()
        body.name = f"obstacle_swall_{self.id}"
        body.pos = self.mean_point.tolist() + [self.height]
        body.quat = scipy.spatial.transform.Rotation.from_euler('z', self.yaw).as_quat(scalar_first=True)
        geom = body.add_geom()
        geom.name = f"obstacle_swall_geom_{self.id}"
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = [self.length / 2, self.width, self.height]
        geom.rgba = [1, 0, 0, 1]
        
    def step(self, model: mujoco.MjModel, delta_t: float):
        """
        Advances the wall's state by a given time step.

        Args:
            model (mujoco.MjModel): The MuJoCo model.
            delta_t (float): The time step by which to advance the wall's state.
        """
        if self.transition_end:
            return
        translation = np.linalg.norm(self.shift_vect)
        direction = self.shift_vect / translation
        if np.dot(direction, self.pos) >= translation:
            self.transition_end = True
            return
        self.pos += direction * delta_t * 10.0
        start_point, end_point = self.start_pos, self.end_pos
        mean_point = (start_point + end_point) / 2
        new_mean_point = mean_point + self.pos
        body = model.body(f"obstacle_swall_{self.id}")
        body.pos = new_mean_point.tolist() + [1.0]

class Map:
    def __init__(self, obstacles: list[Obstacle] = None, extreme_points: np.ndarray = np.array([[0.0,0.0],[10.0,10.0]])):
        """
        Initializes the Map class.

        Args:
            obstacles (list[Obstacle], optional): A list of obstacles. Defaults to None.
            extreme_points (np.ndarray, optional): The extreme points of the map. Defaults to np.array([[0.0,0.0],[10.0,10.0]]).
        """
        if obstacles is None:
            obstacles = []
        self.obstacles: dict[str, list[Obstacle]] = {}
        self.extreme_points = extreme_points
        for obs in obstacles:
            if not isinstance(obs, Obstacle):
                raise ValueError("All obstacles must be instances of Obstacle.")
            l = self.obstacles.setdefault(obs.__class__.__name__, [])
            l.append(obs)
        for k, v in self.obstacles.items():
            v.sort(key=lambda x: x.id)
    
    def get_obstacles(self, name: str) -> list[Obstacle]:
        """
        Returns the list of obstacles of a given type.

        Args:
            name (str): The name of the obstacle type.

        Returns:
            list[Obstacle]: The list of obstacles of the given type.
        """
        return self.obstacles.get(name, [])
    
    def dynamic_obstacles(self) -> list[DynamicObstacle]:
        """
        Returns the list of dynamic obstacles.

        Returns:
            list[DynamicObstacle]: The list of dynamic obstacles.
        """
        res = []
        for obs_list in self.obstacles.values():
            for obs in obs_list:
                if isinstance(obs, DynamicObstacle):
                    res.append(obs)
        return res
    
    def cost(self, pos: np.ndarray) -> float:
        """
        Computes the total cost at a given position due to all obstacles.

        Args:
            pos (np.ndarray): The position for which to compute the cost.

        Returns:
            float: The total computed cost.
        """
        c = 0.0
        for obs_list in self.obstacles.values():
            for obs in obs_list:
                c += obs.cost(pos)
        return c
    
    def add_to_spec(self, model_spec: dict):
        """
        Adds all obstacles to the model specification.

        Args:
            model_spec (dict): The model specification to which the obstacles are added.
        """
        for obs_list in self.obstacles.values():
            for obs in obs_list:
                print(f"Adding obstacle {obs} to model spec.")
                obs.add_to_spec(model_spec)
    
    def step(self, model: mujoco.MjModel, delta_t: float):
        """
        Advances the state of all dynamic obstacles by a given time step.

        Args:
            model (mujoco.MjModel): The MuJoCo model.
            delta_t (float): The time step by which to advance the obstacles' state.
        """
        for obs in self.dynamic_obstacles():
            obs.step(model, delta_t)
        