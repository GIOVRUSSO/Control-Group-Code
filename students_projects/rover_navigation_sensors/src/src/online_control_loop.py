#!/usr/bin/env python
import rospy

from waypoint_follower_node import *
from cost_cache import *
from klc_controller import *

from state_manager_node import StateManager


cache = CostCache()

state_manager = StateManager()

target = [6.5, 3.5]
mode = 0

control_frame = 'husky2_tf/base_link'
reference_frame = 'husky2_tf/map'

#klc = ControllerKLCOnline(target, mode)
klc = ControllerKLC(target, mode)
#mx, my, htime, sx, sy = klc.update()

mx, my, htime, sx, sy = klc.update()
wp_x, wp_y = cache.get_targets()

np.save("mean_stdv_plot_data", np.array([mx, sx, my, sy]))

#cache.set_next_target(mx, my)

print("Got waypoints. Starting to follow the planned trajectory...")
waypoint_follower = WaypointFollowerNode()

result = waypoint_follower.follow_waypoints([wp_x,wp_y], target)

while result != 0 or result != 1:
    current_position = state_manager.get_2D_position()
    # new_target_x = target[0] - current_position[0]
    # new_target_y = target[1] - current_position[1]
    # new_target = [new_target_x, new_target_y]
    
    # print("New target: ", new_target)
    #klc.set_goal(new_target)
    klc.set_zmin([int(current_position[0]), int(current_position[1])])
    klc.extract_passive_dynamics(mode)
    klc.compute_state_vect()
    mx, my, htime, sx, sy = klc.update()
    wp_x, wp_y = cache.get_targets()
    print("Waypoints: ", wp_x, wp_y)
    result = waypoint_follower.follow_waypoints([wp_x,wp_y], target)

if result == 0:
    rospy.loginfo("Last waypoint reached. Target achieved")
elif result == 1:
    rospy.loginfo("Mission failed. Target not achieved")
