import rospy
from geometry_msgs.msg import Twist
# import sys, select, os
  
from datetime import datetime

from utility import *
import numpy as np

import pygame


### Pygame stuff
pygame.init()

SCREEN_WIDTH = 100
SCREEN_HEIGHT = 80

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

### ROS stuff
init_pos = get_position()
log = [get_actual_position(init_pos)[0:2]]


LIN_VEL_LIMIT = 2.0 # m/s
ANG_VEL_LIMIT = 1.2 # rads
LIN_VEL_STEP  = 0.2
ANG_VEL_STEP  = 0.1

lin_vel = 0.0
ang_vel = 0.0

rospy.init_node('husky_teleop_keyboard')

rospy.set_param('husky_teleop_topic', '/husky1/husky_velocity_controller/cmd_vel')
topic = rospy.get_param('husky_teleop_topic')

cmd_vel_pub = rospy.Publisher(topic, Twist, queue_size=10)

###ROS functions
def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def bound_lin_vel(lin_vel_cmd):
    lin_vel_cmd = constrain(lin_vel_cmd, -LIN_VEL_LIMIT, LIN_VEL_LIMIT)
    return lin_vel_cmd

def bound_ang_vel(ang_vel_cmd):
    ang_vel_cmd = constrain(ang_vel_cmd, -ANG_VEL_LIMIT, ANG_VEL_LIMIT)
    return ang_vel_cmd

def generate_cmd_vel_msg(lin_vel_cmd, ang_vel_cmd):
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x  = lin_vel_cmd
    cmd_vel_msg.linear.y  = 0.0
    cmd_vel_msg.linear.z  = 0.0
    cmd_vel_msg.angular.x = 0.0
    cmd_vel_msg.angular.y = 0.0
    cmd_vel_msg.angular.z = ang_vel_cmd
    return cmd_vel_msg


vel_log = []
run = True
while run:
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        lin_vel = bound_lin_vel(lin_vel + LIN_VEL_STEP)
    elif keys[pygame.K_s]:# keyboard.is_pressed('s'):
        lin_vel = bound_lin_vel(lin_vel - LIN_VEL_STEP)
    else:
        lin_vel = 0.25*lin_vel
    if keys[pygame.K_a]: #keyboard.is_pressed('a'):
        ang_vel = bound_ang_vel(ang_vel + ANG_VEL_STEP)
    elif keys[pygame.K_d]: #keyboard.is_pressed('d'):
        ang_vel = bound_ang_vel(ang_vel - ANG_VEL_STEP)
    else:
        ang_vel = 0.25*ang_vel
        
    cmd_vel_msg = generate_cmd_vel_msg(lin_vel, ang_vel)
    vel_log.append(lin_vel)
    # np.save('VL.npy',vel_log)
    cmd_vel_pub.publish(cmd_vel_msg)
    pos = get_actual_position(init_pos)
    log.append(pos[0:2])
    # Genera la data e l'ora attuale nel formato desiderato


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

current_time = datetime.now().strftime("%Y%m%d_%H%M%S")

# Salva il file con la data e l'ora nel nome
np.save(f'log_position_{current_time}.npy', np.array(log))
pygame.quit()
