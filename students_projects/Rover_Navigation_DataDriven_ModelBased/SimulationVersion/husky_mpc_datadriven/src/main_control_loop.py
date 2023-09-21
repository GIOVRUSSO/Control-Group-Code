import rospy
from mpc_controller import *
from cost_cache import *
from klc_controller_online import *

cache = CostCache()

target = [8, 8]
mode = 0
klc = ControllerKLCOnline(target, mode)
mx, my, htime, sx, sy = klc.update()
#np.save("mean_stdv_plot_data", np.array([x, sx, y, sy]))
klc.export_metrics(mx, my, htime)
cache.set_next_target(mx, my)

plt.rcParams.update({'font.size': 30})
dur = 30

#PLOT X
x = np.array([x for x in range(dur)])
y = np.array(mx)
ci = np.array(sx)   
fig, ax = plt.subplots()
plt.xlim([0, dur])
ax.plot(x,y, label="Planning x")
plt.xlabel("Time")
plt.ylabel("State")
plt.title("Position on x")
ax.fill_between(x, (y-ci), (y+ci), color='b', alpha=.1)
plt.savefig('/home/marco/Desktop/klc_dinamica_passiva' + str(mode) + '/xKLC_PM_online_real.png')

#PLOT Y
x = np.array([x for x in range(dur)])
y = np.array(my)
ci = np.array(sy)
fig, ax = plt.subplots()
plt.xlim([0, dur])
ax.plot(x,y, label="Planning y")
plt.xlabel("Time")
plt.ylabel("State")
plt.title("Position on y")
ax.fill_between(x, (y-ci), (y+ci), color='b', alpha=.1)
plt.savefig('/home/marco/Desktop/klc_dinamica_passiva' + str(mode) + '/yKLC_PM_online_real.png')

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

print("saving simulation results!")
np.save("klc_online_real_simulation_" + str(mode), np.array([mpc_x_history, mpc_y_history, mpc_t_history]))

u1, u2 = mpc.get_inputs()

np.save("inputs_for_husky_real_klc_online_"+ str(mode), np.array([u1, u2]))