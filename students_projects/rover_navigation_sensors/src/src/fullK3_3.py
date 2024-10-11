import rospy
from mpc_controller_3_3 import *
from cost_cache import *
from klc_controller import *
#from scipy import interpolate
import matplotlib.pyplot as plt

cache = CostCache()

mode = 0

targetx = 0.3*np.load('./waypoints2/wx_3_3.npy')
targety = 0.3*np.load('./waypoints2/wy_3_3.npy')


targetx = np.append(targetx,[targetx[-1],targetx[-1],targetx[-1],targetx[-1],targetx[-1],targetx[-1],targetx[-1],targetx[-1],targetx[-1],targetx[-1]])
targety = np.append(targety,[targety[-1],targety[-1],targety[-1],targety[-1],targety[-1],targety[-1],targety[-1],targety[-1],targety[-1],targety[-1]])


target = [targetx[-1],targety[-1]] #target position, needs to be updated for each scenario. 

num_waypoints = targetx.shape[0]

#print('target x is',targetx)

#global length
length = 150

indices = np.linspace(0,num_waypoints-1,length)
targetx_in = np.interp(indices,np.arange(num_waypoints),targetx)
targety_in = np.interp(indices,np.arange(num_waypoints),targety)

#print(targetx_in)

cache.set_next_target(targetx_in,targety_in) #Waypoint folder in ./catkin_ws/src/husky_mpc_datadriven_src/
#Change line above to set waypoints

#target = [cache.target_x, cache.target_y]
#print('target=',target)

mpc = ControllerMPC([0,0,0])

mpc_x_history = np.array([])
mpc_y_history = np.array([])
mpc_t_history = np.array([])
mpc_t = 0

while not rospy.is_shutdown():   
    mpc_x, mpc_y, stopping_cond = mpc.update(target)
    mpc_x_history = np.append(mpc_x_history, mpc_x)
    mpc_y_history = np.append(mpc_y_history, mpc_y)
    mpc_t += 0.1
    mpc_t_history = np.append(mpc_t_history, mpc_t)
    if stopping_cond == 1:
        break
        
u1, u2 = mpc.get_inputs()
#cost = mpc.get_objective()

#np.save('cost.npy',cost)
#plt.plot(cost)
plt.show()
np.save('u1.npy',u1)
np.save('u2.npy',u2)
