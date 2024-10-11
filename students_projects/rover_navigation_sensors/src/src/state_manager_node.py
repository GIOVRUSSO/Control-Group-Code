#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry


"""
This class is responsible for subscribing to the estimated state of the robot and providing the current state to other nodes.
It subscribes to the /odometry/filtered/global topic to get the estimated state of the robot.
"""
class StateManager():
    """
    This method initializes the StateManager class.
    It sets the robot_namespace parameter to 'husky2'.
    """
    def __init__(self):
        rospy.set_param('robot_namespace', 'husky2')
        
        self._position2D = [0.0, 0.0]  # Initialize _state as a 2D vector
        self._old_position2D = [0.0, 0.0]  # Initialize _old_state as a 2D vector
        
        self._pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._old_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self._updating = False
        self._first_update = False
        
        self.robot_namespace = rospy.get_param('robot_namespace')
        
        # rospy.init_node('state_subscriber', anonymous=True)  # Uncomment this if rospy is not initialized elsewhere
        self.state_subscriber = rospy.Subscriber(self.robot_namespace + '/odometry/filtered/global', Odometry, self.get_estimated_state_callback)
    
    """
    This method is a callback function that is called when a new message is received on the /odometry/filtered/global topic.
    It updates the _state and _old_state variables with the new and old state of the robot, respectively.
    
    :param data: The Odometry message received on the /odometry/filtered/global topic.
    """
    def get_estimated_state_callback(self, data):
        self._updating = True
        self._first_update = True
        position_x = data.pose.pose.position.x
        position_y = data.pose.pose.position.y
        
        # Update _old_state before updating _state
        self._old_position2D = self._position2D[:]
        self._position2D = [position_x, position_y]
        
        self._pose = data.pose.pose
        self._updating = False

    """
    This method returns the 2D position of the robot.
    """
    def get_2D_position(self):
        if self._updating:
            return self._old_position2D
        else:
            return self._position2D
    """
    This method returns the estimated state of the robot: position and orientation.
    
    :return: The estimated state of the robot.
    """   
    def get_pose(self):
        if self._updating:
            return self._old_pose
        else:
            return self._pose
    
    """
    This method checks if the state of the robot has been updated at least once.
    
    :return: True if the state has been updated at least once, False otherwise.
    """
    def is_valid_state(self):
        return self._first_update
    
if __name__ == '__main__':
    rospy.init_node('state_manager_node', anonymous=True)
    state_manager = StateManager()
    rospy.spin()
