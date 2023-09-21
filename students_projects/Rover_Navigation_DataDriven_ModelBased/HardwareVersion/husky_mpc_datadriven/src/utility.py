# Import ros package:
import rospy
from geometry_msgs.msg import Twist
import tf
import tf2_ros
from tf import transformations as t
import numpy as np
from gazebo_msgs.srv import GetLinkState 
import geometry_msgs.msg

def get_link_states(link_name, reference_frame):
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        state = get_link_state(link_name, reference_frame)
        return state
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def get_position():
    T_odom_world = get_link_states('base_link', 'odom') #its the same as T_baselink_world
    #rospy.Subscriber('/vrpn_client_ros/RigidBody/pose', Pose, some_callback)
    
    print(T_odom_world)
    trans = tf.transformations.translation_from_matrix(T_odom_world)
    rot = tf.transformations.quaternion_from_matrix(T_odom_world)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)

    return [trans[0], trans[1], trans[2], roll, pitch, yaw]

def get_actual_position(init_position):

    actual_position = get_position()

    result = np.dot(t.inverse_matrix(init_position),actual_position)
    trans = tf.transformations.translation_from_matrix(result)
    rot = tf.transformations.quaternion_from_matrix(result)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)

    return [trans[0], trans[1], trans[2], roll, pitch, yaw]


def get_obstacle_position_odom(init_position, obs_position_world):

    obs_pos = t.concatenate_matrices(t.translation_matrix([obs_position_world[0], obs_position_world[1], obs_position_world[2]]),
            t.quaternion_matrix(([obs_position_world[3], obs_position_world[4], obs_position_world[5], obs_position_world[6]])))

    result = np.dot(t.inverse_matrix(init_position),obs_pos)
    trans = tf.transformations.translation_from_matrix(result)
    rot = tf.transformations.quaternion_from_matrix(result)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)

    return [trans[0], trans[1], trans[2], roll, pitch, yaw]


def publish_tf(translation, rot):
    br = tf2_ros.TransformBroadcaster()

    tr = geometry_msgs.msg.TransformStamped()

    tr.header.stamp = rospy.Time.now()
    tr.header.frame_id = "odom"
    tr.child_frame_id = "base_link_gazebo"
    tr.transform.translation.x = translation[0]
    tr.transform.translation.y = translation[1]
    tr.transform.translation.z = 0.0
    
    rotation = t.quaternion_from_euler(rot[0], rot[1], rot[2])

    tr.transform.rotation.x = rotation[0]
    tr.transform.rotation.y = rotation[1]
    tr.transform.rotation.z = rotation[2]
    tr.transform.rotation.w = rotation[3]

    br.sendTransform(t) 


def add_noise_to_states(states):
    noise_xy = 0*np.random.normal(0,1,1)
    noise_theta = 0*np.random.normal(0,1,1)

    return [states[0] + noise_xy, states[1] + noise_xy, states[2] + noise_theta]


def print_states(x, y, z):
    print("(x = " + str(x) + ", y = " + str(y) + ", theta = " + str(z) + ")")
