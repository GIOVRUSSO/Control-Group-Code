import rospy
import numpy as np
from cost_cache import *
from utility import *
from gazebo_msgs.srv import GetModelState

"""
Obstacle: A class representing obstacles in a robot's environment.

This class retrieves obstacle poses from Gazebo and processes them to provide obstacle information.
"""

class Obstacle:

    NOBSTACLES = 7

    """
    Initialize the Obstacle class and retrieve obstacle poses from Gazebo.
    
    :param self: The instance of the class.
    """
    def __init__(self):

        #Get obstacles from parameter server
        self.cache = CostCache()
        self.obs_odom = []
        #self.num = rospy.get_param("/number_of_obstacles")

        self.get_pose_from_gazebo()

        """for obs in self.obs_odom:
            print(obs)"""


    """
    Retrieve obstacle poses from Gazebo using the GetModelState service.
    
    :param self: The instance of the class.
    """
    def get_pose_from_gazebo(self):

        rospy.wait_for_service('/gazebo/get_model_state')
        
        for i in range(self.NOBSTACLES):
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            obstacle = "obstacle" + str(i)
            param = get_model_state(obstacle , "")
            pose = [param.pose.position.x, param.pose.position.y, param.pose.position.z, param.pose.orientation.x, param.pose.orientation.y, param.pose.orientation.z, param.pose.orientation.w]
            self.obs_odom.append(get_obstacle_position_odom(self.cache.get_T(), pose) + [1, 1])
        
        
    """
    Get the list of obstacle poses.
    
    :param self: The instance of the class.
    :return: A list of obstacle poses.
    """
    def get_obs(self):
        return self.obs_odom