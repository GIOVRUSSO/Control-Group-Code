import rospy
import numpy as np
from cost_cache import *
from utility import *
from tf2_geometry_msgs import do_transform_pose
from gazebo_msgs.srv import GetModelState
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped
from point_cloud_perception.src.utils import get_transform
from state_manager_node import StateManager

"""
This class is responsible for managing obstacles. 
It subscribes to the obstacle cluster centroids topic and stores the obstacles in a dictionary.
It also provides a method to check if the obstacles have changed.
It takes into account the robot's namespace and tf_prefix.
It publishes obstacle centroids as markers in RViz.
"""
class ObstacleManager:

    """
    This method initializes the ObstacleManager class.
    It subscribes to the obstacle cluster centroids topic and initializes the obstacle dictionaries.
    It initializes the publisher to publish obstacle centroids as markers in RViz.
    It sets the robot's namespace and tf_prefix and the sensor and reference frames.
    """
    def __init__(self):
        
        rospy.set_param('robot_namespace', 'husky2')
        
        self.cache = CostCache()
        self.obs_odom = {}  
        self.obs_odom_old = {}  
        self._updating = False
        self.wait_for_initial_obstacle = False
        
        self._state_manager = StateManager()
        
        self.obstacle_subscriber = rospy.Subscriber("/obstacle_cluster_centroids", MarkerArray, self._obstacle_cluster_callback_t)
        
        self.obstacle_publisher = rospy.Publisher("/obs", Marker, queue_size=10)
        
        self._robot_namespace = rospy.get_param('robot_namespace')
        self._tf_prefix = self._robot_namespace + "_tf"
        
        self._sensor_frame = self._tf_prefix + "/velodyne"
        self._reference_frame = self._tf_prefix + "/odom"
    
    """
    This method gets the pose of the obstacles from Gazebo and stores them in the obstacle dictionary. It doesn't retrieve obstacle positions from the topic.
    """
    def get_pose_from_gazebo(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        for i in range(self.NOBSTACLES):
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            obstacle = "obstacle" + str(i)
            param = get_model_state(obstacle , "")
            pose = [param.pose.position.x, param.pose.position.y, param.pose.position.z, 
                    param.pose.orientation.x, param.pose.orientation.y, param.pose.orientation.z, param.pose.orientation.w]
            self.obs_odom.append(get_obstacle_position_odom(self.cache.get_T(), pose) + [1, 1])
    
    """
    This method returns the list of obstacles.
    """
    def get_obs(self):
        if self._updating:
            return list(self.obs_odom_old.values())  
        else:
            return list(self.obs_odom.values())
        
    """
    This method clears the obstacle dictionary.
    """
    def clear_obstacles(self):

        self.obs_odom = {}


    """
    This method adds an obstacle to the obstacle dictionary by using the rounded pose as the key. This serves for avoiding to add the same obstacle multiple times or to add too near onstacle.

    :param pose: The pose of the obstacle.
    """
    def add_obstacle(self, pose):

        key = (round(pose[0]), round(pose[1]))  # Arrotonda le coordinate
        self.obs_odom[key] = pose  # Memorizza il resto delle informazioni come valore
        
        #self.publish_obstacle(pose)
            
    """
    This method is the callback for the obstacle cluster centroids subscriber. 
    It receives the obstacle cluster centroids, compute their position wrt to map frame and stores them in the obstacle dictionary.

    :param msg: The obstacle cluster centroids message.
    """       
    def _obstacle_cluster_callback_t(self, msg):

        self._updating = True
        self.obs_odom_old = self.obs_odom.copy()  # Mantieni una copia degli ostacoli precedenti
        self.clear_obstacles()
        
        velodyne_to_map_transform = get_transform(self._sensor_frame, self._reference_frame)
        if velodyne_to_map_transform is None:
            rospy.logerr("Failed to get transform from" + self._sensor_frame + "to" + self._reference_frame)
            return
        
        i = 0
        for marker in msg.markers:
            obs_pose = PoseStamped()
            obs_pose.pose = marker.pose
            obs_pose.header = marker.header
            
            obs_pose = do_transform_pose(obs_pose, velodyne_to_map_transform)
            obs_pose.header.frame_id = self._reference_frame
            
            pose_transformation = [
                obs_pose.pose.position.x,
                obs_pose.pose.position.y,
                obs_pose.pose.position.z,
                obs_pose.pose.orientation.x,
                obs_pose.pose.orientation.y,
                obs_pose.pose.orientation.z,
                obs_pose.pose.orientation.w,
                1, 1
            ]
            
            self.add_obstacle(pose_transformation)
        
        self._updating = False
        self.wait_for_initial_obstacle = True
        self.updated_scene = False
        
    """
    This method checks if the obstacles have changed. It compares the current obstacle dictionary with the previous one and returns True if the obstacles have changed.
    It also checks if the changed obstacle is near the robot or it can be ignored.

    :return: True if the obstacles have changed, False otherwise.
    """
    def has_obstacles_changed(self):

        while self._updating:
            pass
        
        if not self.obs_odom or not self.obs_odom_old:
            return False
        
        # if len(self.obs_odom) != len(self.obs_odom_old):
        #     print("Different number of obstacles")  
        #     return True
        
        for new_obs in self.obs_odom:
            if new_obs not in self.obs_odom_old and np.linalg.norm(np.array(new_obs[:2] - np.array(self._state_manager.get_2D_position()))) <= 0.45:
                print("Different position of obstacles")
                return True
        
        return False

    """
    This method verifies that the obstacle positions have been detected at least once to avoid that the control algorithm starts without obstacles initially.

    :return: True if the obstacle positions have been detected at least once, False otherwise.
    """    
    def check_update(self):
        return self.wait_for_initial_obstacle
    
    
    """
    This method publishes the obstacle as a marker in RViz.

    :param pose: The pose of the obstacle.
    """
    def publish_obstacle(self,pose):
        
        marker = Marker()
        marker.ns = "obs"
        marker.header.frame_id = self._reference_frame
        marker.header.stamp = rospy.Time.now()
        marker.id = 8
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        
        marker.pose.orientation.x = pose[3]
        marker.pose.orientation.y = pose[4]
        marker.pose.orientation.z = pose[5]
        marker.pose.orientation.w = pose[6]
        
        marker.scale.x = 0.5  # Diameter of the sphere
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        marker.color.a = 1.0  
        marker.color.r = 0.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 1.0
        
        self.obstacle_publisher.publish(marker)
