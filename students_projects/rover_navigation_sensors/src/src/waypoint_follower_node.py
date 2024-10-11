#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
import tf
from scipy.interpolate import UnivariateSpline

from state_manager_node import StateManager
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline

from tf2_geometry_msgs import do_transform_pose
from point_cloud_perception.src.utils import get_transform

from obstacle_manager import ObstacleManager

from cost_cache import CostCache

"""
WaypointFollowerNode: ROS node for following a sequence of waypoints.

"""
class WaypointFollowerNode():

    """
    This method initializes the WaypointFollowerNode class.
    It sets the parameters for the robot namespace, the waypoint topic, the reference frame and the control frame.
    It waits for the move_base server to be up and running.
    """
    def __init__(self):
        rospy.set_param('robot_namespace', 'husky2')
        rospy.set_param('waypoint_topic', "/wp")
        
        rospy.set_param('reference_frame', 'map')
        rospy.set_param('control_frame', 'base_link')
        
        self.waypoint_topic = rospy.get_param('waypoint_topic')
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, Marker, queue_size=1)
        self.id = 0

        self._state_manager = StateManager()
        self._obstacle_manager = ObstacleManager()

        self._robot_namespace = rospy.get_param('robot_namespace')
        self._tf_prefix = self._robot_namespace + "_tf"
        
        
        self._reference_frame = self._tf_prefix + "/" + rospy.get_param('reference_frame')
        self._control_frame = self._tf_prefix + "/" + rospy.get_param('control_frame')
        
        self.cache = CostCache()
        
        self.move_base_client = actionlib.SimpleActionClient('husky2/move_base', MoveBaseAction)
        print("Waiting for move_base server")
        self.move_base_client.wait_for_server()

    
    """
    This methos is responsible for following a single waypoint.
    It sends the goal to the move_base server and waits for the result.
    If the goal is not achieved within 10 seconds, it cancels the goal.
    
    :param goal: the goal to be reached
    :return: True if the goal is achieved, False otherwise
    """
    def go_to_waypoint(self, goal):
        # print("Going to waypoint")
        target = MoveBaseGoal()
        target.target_pose = goal
        target.target_pose.header.stamp = rospy.Time.now()
        target.target_pose.header.frame_id = self._reference_frame

        server_up = self.move_base_client.wait_for_server()
        if server_up:
            self.move_base_client.send_goal(target)
            finished_within_time = self.move_base_client.wait_for_result(timeout=rospy.Duration(10.0))
            if not finished_within_time:
                self.move_base_client.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    return True
                elif state == GoalStatus.PREEMPTED:
                    rospy.loginfo("Goal pre-empted!")
                else:
                    rospy.loginfo(self.move_base_client.get_result())
        else:
            rospy.loginfo("Cannot send goal to move_base. Server down")
        
        return False

    """
    This method filters a sequence of waypoints by removing those that are too close.
    
    :param waypoints: the sequence of waypoints to be filtered
    :param distance_threshold: the minimum distance between two waypoints
    
    :return: the filtered sequence of waypoints
    """
    def filter_waypoints(self, waypoints, distance_threshold=0.5):
        """
        Filtra i waypoint eliminando quelli troppo vicini o troppo distanti dalla traiettoria.
        """
        x = waypoints[0]
        y = waypoints[1]
        
        filtered_x = [x[0]]
        filtered_y = [y[0]]

        for i in range(1, len(x)):
            # Controllo la distanza tra il waypoint corrente e tutti i waypoint filtrati
            add_waypoint = True
            for j in range(len(filtered_x)):
                dist = np.sqrt((x[i] - filtered_x[j]) ** 2 + (y[i] - filtered_y[j]) ** 2)
                if dist < distance_threshold:
                    add_waypoint = False
                    break
            
            if add_waypoint:
                filtered_x.append(x[i])
                filtered_y.append(y[i])
        
        filtered_x[-1] = x[-1]
        filtered_y[-1] = y[-1]

        return np.array([filtered_x, filtered_y])

    """
    This method is responsible for following a sequence of waypoints.
    It filters the waypoints, interpolates them using cubic splines and sends the goal one by one to the move_base server until the last waypoint is reached.
    """
    def follow_waypoints(self, waypoints, desired_target):
        #print("Following waypoints")
        
        filtered_wps = self.filter_waypoints(waypoints)
        
        #print("Original number of waypoints: ", len(waypoints[0]))
        
        # 2. Interpolation with cubic splines
        interpolated_wps, tangents = self.cubic_spline_interp(filtered_wps)

        self.cache.set_targets(interpolated_wps[0], interpolated_wps[1])
        
        #self.plot_waypoints(interpolated_wps, tangents, 'Interpolated Waypoints')
        
        num_waypoints = len(interpolated_wps[0])
        #print("Interpolated number of waypoints: ", num_waypoints)

        for i in range(2,num_waypoints):
            angle = tangents[i]  
            q = self.quaternion_from_orientation(angle)

            pose = PoseStamped()
            
            # base_frame_to_map_transform = get_transform(self._control_frame, self._reference_frame)
            # if base_frame_to_map_transform is None:
            #     rospy.logerr("Failed to get transform from" + self._control_frame + "to" + self._reference_frame)
            #     return
            
            
            pose.header.frame_id = self._reference_frame
            pose.header.seq = i
            pose.pose.position.x = interpolated_wps[0][i]
            pose.pose.position.y = interpolated_wps[1][i]
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            # Transform the pose to the control frame
            # pose = do_transform_pose(pose, base_frame_to_map_transform)
            
            if self._obstacle_manager.has_obstacles_changed():
                state = self.move_base_client.get_state()
                if state != GoalStatus.SUCCEEDED and state != GoalStatus.PREEMPTED:
                    self.move_base_client.cancel_goal()
                    rospy.loginfo("Goal cancelled due to obstacle change")
                    return 2

            self.publish_waypoint(pose, pose.header,0)
            print ("Publised waypoint:",pose.pose.position.x,pose.pose.position.y)
                
            if not self.go_to_waypoint(pose):
                return 2

        last_wp = [interpolated_wps[0][-1], interpolated_wps[1][-1]]
        if np.linalg.norm(np.array(desired_target) - np.array(last_wp)) < 0.15:
            return 0
        else:
            return 1

    """
    This method interpolates a sequence of waypoints using cubic splines.
    
    :param waypoints: the sequence of waypoints to be interpolated
    
    :return: the interpolated waypoints and the angular coefficients of the tangents to the curve at each waypoint.
    """
    def cubic_spline_interp(self, waypoints):
        x = waypoints[0]
        y = waypoints[1]

        if len(x) < 2:
            raise ValueError('At least two waypoints are required for interpolation')

        # Cubic spline interpolation
        cs_x = CubicSpline(range(len(x)), x)
        cs_y = CubicSpline(range(len(y)), y)

        # Parametrizzazione per creare un numero di punti interpolati
        n = len(x)*3
        t_new = np.linspace(0, len(x) - 1, n)
        interp_x = cs_x(t_new)
        interp_y = cs_y(t_new)

        # Calcolo delle tangenti come derivate della spline cubica
        tangents = np.arctan2(cs_y(t_new, 1), cs_x(t_new, 1))

        return np.array([interp_x, interp_y]), tangents
    
    """
    This method interpolates a sequence of waypoints using linear interpolation.
    
    :param waypoints: the sequence of waypoints to be interpolated
    
    :return: the interpolated waypoints and the angular coefficients of the tangents to the curve at each waypoint.
    """
    def linear_interp(self, waypoints):
        x = waypoints[0]
        y = waypoints[1]

        if len(x) < 2:
            raise ValueError('At least two waypoints are required for interpolation')

        # Interpolazione lineare
        n = len(x) * 3
        t_new = np.linspace(0, len(x) - 1, n)
        interp_x = np.interp(t_new, range(len(x)), x)
        interp_y = np.interp(t_new, range(len(y)), y)

        # Calcolo delle tangenti come derivate approssimative delle coordinate interpolate
        dx = np.gradient(interp_x)
        dy = np.gradient(interp_y)
        tangents = np.arctan2(dy, dx)

        return np.array([interp_x, interp_y]), tangents

    """
    This method interpolates a sequence of waypoints using UnivariateSpline. This allow to obtain a curve that is smooth and not necessary passes through all the waypoints.
    
    :param waypoints: the sequence of waypoints to be interpolated
    :param smoothing_factor: the smoothing factor for the spline
    
    :return: the interpolated waypoints and the angular coefficients of the tangents to the curve at each waypoint.
    """
    def interpolate(self, waypoints, smoothing_factor=3):
        x = waypoints[0]
        y = waypoints[1]

        if len(x) < 2:
            raise ValueError('At least two waypoints are required for interpolation')

        # Spline lisciata con UnivariateSpline e parametro di lisciatura 's'
        spline_x = UnivariateSpline(range(len(x)), x, s=smoothing_factor)
        spline_y = UnivariateSpline(range(len(y)), y, s=smoothing_factor)

        # Parametrizzazione per creare un numero di punti interpolati
        n = len(x) * 3
        t_new = np.linspace(0, len(x) - 1, n)

        # Valori interpolati
        interp_x = spline_x(t_new)
        interp_y = spline_y(t_new)

        # Calcolo delle tangenti come derivate della spline cubica
        tangents = np.arctan2(spline_y.derivative()(t_new), spline_x.derivative()(t_new))

        return np.array([interp_x, interp_y]), tangents

    """
    This method converts an orientation angle to a quaternion.
    
    :param theta: the orientation angle
    
    :return: the quaternion corresponding to the orientation angle
    """
    def quaternion_from_orientation(self, theta):
        qw = np.cos(theta / 2)
        qx = 0
        qy = 0
        qz = np.sin(theta / 2)
        return np.array([qx, qy, qz, qw])

    """
    This method publish a waypoint as a marker in the RViz environment.
    
    :param wp_pose: the pose of the waypoint
    :param header: the header of the waypoint
    :param id: the id of the waypoint
    """
    def publish_waypoint(self, wp_pose, header,id):
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker.ns = "waypoint"
        marker.header = header
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = wp_pose.pose
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.waypoint_pub.publish(marker)

    
    """
    This method plots the waypoints and the tangents to the curve at each waypoint on a 2D plot starting from robot's current position.
    """
    def plot_waypoints(self, waypoints, tangents, label):
        plt.figure(figsize=(10, 10))
        x = waypoints[0]
        y = waypoints[1]

        plt.scatter(x, y, color='red', label= label)

        for i in range(len(x)):
            angle = tangents[i]
            dx = np.cos(angle) * 0.2
            dy = np.sin(angle) * 0.2
            plt.arrow(x[i], y[i], dx, dy, head_width=0.05, color='green', label='Orientamento' if i == 0 else '')

        plt.title('Waypoints e Orientamenti')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

if __name__ == '__main__':
    
    rospy.init_node('waypoint_follower_node')
    Result = WaypointFollowerNode()
    rospy.spin()
