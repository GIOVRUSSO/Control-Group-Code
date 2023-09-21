import rospy
from mpc_controller import *
from cost_cache import *
from klc_controller_online import *

cache = CostCache()

rospy.init_node('husky', anonymous=True)

target = [8, 8]
klc = ControllerKLCOnline(target, 0)
x, y, time = klc.update()
klc.export_metrics(x, y, time)
cache.set_next_target(x, y)

mpc = ControllerMPC([0,0,0])

mpc_x_history = np.array([])
mpc_y_history = np.array([])
mpc_t_history = np.array([])
mpc_t = 0

sub = tf.TransformListener()
        # Initialize ROS node
pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        # Create a Twist message for robot motion
move_cmd = Twist()

rate = rospy.Rate(10)


while not rospy.is_shutdown():   

    try:
        (trans,rot) = sub.lookupTransform('/odom', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)

    u, mpc_x, mpc_y, stopping_cond = mpc.update(target, [trans[0], trans[1], yaw])
    mpc_x_history = np.append(mpc_x_history, mpc_x)
    mpc_y_history = np.append(mpc_y_history, mpc_y)
    mpc_t += 0.1
    mpc_t_history = np.append(mpc_t_history, mpc_t)
    if stopping_cond == 1:
        break

    print(u)

    # Set the linear and angular velocities for the robot's motion
    move_cmd.linear.x = u[0] 
    move_cmd.angular.z = u[1]

        # Publish the motion command
    pub.publish(move_cmd)

        # Sleep according to the defined rate
    rate.sleep()


print("salvataggio dei risultati nella simulazione!")
np.save("klc_vision_linear_results_from_simulation", np.array([mpc_x_history, mpc_y_history, mpc_t_history]))