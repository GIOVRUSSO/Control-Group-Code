#!/usr/bin/env python

import rospy
from waypoint_follower_node import WaypointFollowerNode
from cost_cache import *
from klc_controller import *

import numpy as np

import matplotlib.pyplot as plt



cache = CostCache()

target = [6.5, 3.5]
mode = 0

#klc = ControllerKLCOnline(target, mode)
klc = ControllerKLC(target, mode)
#mx, my, htime, sx, sy = klc.update()

mx, my, htime, sx, sy, wp_x, wp_y = klc.update()


np.save("mean_stdv_plot_data", np.array([mx, sx, my, sy]))
klc.export_metrics(mx, my, htime)
#cache.set_next_target(mx, my)

dur = 30

waypoint_follower = WaypointFollowerNode()

waypoint_follower.follow_waypoints([wp_x,wp_y], target)

klc.export_metrics(mx, my, htime)


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
plt.savefig('/home/adamo/catkin_ws/src/rover_navigation/src/klc_passive_dynamic/' + str(mode) + '/xKLC.png')

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
plt.savefig('/home/adamo/catkin_ws/src/rover_navigation/src/klc_passive_dynamic/' + str(mode) + '/yKLC.png')







