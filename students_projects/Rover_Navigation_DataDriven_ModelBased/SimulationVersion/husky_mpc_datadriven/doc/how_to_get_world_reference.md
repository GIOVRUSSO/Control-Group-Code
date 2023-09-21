# How to get world reference to [0, 0, 0]

1. Compute the trasformation matrix $\,T^{base\_link}_{world} = T^{odom}_{world}$ (only for the spawn instant).
     - Get the value of **position** and the **orientation** of **base_link** of the robot respect to **world** frame:\
     ```
     rosservice call /gazebo/get_link_state "link_name: 'base_link' reference_frame: 'world'"
     ```
     - The result of this service call is:
     ```
     link_state: 
        link_name: "base_link"
        pose: 
            position: 
            x: -6.991883085173431
            y: -7.00054321876081
            z: 0.18227865904411014
            orientation: 
            x: 1.1163616387520356e-05
            y: 6.949258443087348e-06
            z: 2.5143568295715658e-05
            w: 0.9999999995974412
        twist: 
            linear: 
            x: 4.1692257868217095e-05
            y: 5.3699901153160035e-06
            z: 5.00429446487284e-06
            angular: 
            x: -9.025332930442334e-06
            y: -2.741401415033194e-06
            z: 1.5814206152941707e-06
        reference_frame: "world"
        success: True
        status_message: "GetLinkState: got state"
    ```
    - To compute a matrix in ROS Python i need to use ServiceProxy and wait_for_service in a new method **get_link_states**:
    ```python
    def get_link_states(link_name, reference_frame):
        rospy.wait_for_service('/gazebo/get_link_state')
        try:
            get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            state = get_link_state(link_name, reference_frame)
            return state
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    ```
    - To create the matrix
    ```python
    T_odom_world = get_link_states('husky::base_link', 'world') 
    T_O_W = t.concatenate_matrices(t.translation_matrix([T_odom_world.link_state.pose.position.x, T_odom_world.link_state.pose.position.y, T_odom_world.link_state.pose.position.z]),
                                t.quaternion_matrix([T_odom_world.link_state.pose.orientation.x, T_odom_world.link_state.pose.orientation.y, T_odom_world.link_state.pose.orientation.z, T_odom_world.link_state.pose.orientation.w]))
    ```
2. After i'm able to get transformation matrices, in the control low, i can get the new $T^b_w$ matrix and reference it to the initial position matrix which i get in the initialization of the code:
```python
new_pose = get_link_states('husky::base_link', 'world') 
    new_T_O_W = t.concatenate_matrices(t.translation_matrix([new_pose.link_state.pose.position.x, new_pose.link_state.pose.position.y, new_pose.link_state.pose.position.z]),
                                t.quaternion_matrix([new_pose.link_state.pose.orientation.x, new_pose.link_state.pose.orientation.y, new_pose.link_state.pose.orientation.z, new_pose.link_state.pose.orientation.w]))

    new_real_pose = np.dot(t.inverse_matrix(T_O_W),new_T_O_W)

    trans = tf.transformations.translation_from_matrix(new_real_pose)
    rot = tf.transformations.quaternion_from_matrix(new_real_pose)
```