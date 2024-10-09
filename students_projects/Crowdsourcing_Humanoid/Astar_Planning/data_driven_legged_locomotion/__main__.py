from datetime import datetime
import matplotlib.pyplot as plt
import mediapy as media
import numpy as np
import pathlib
import logging
import mujoco
import mujoco.viewer as viewer
import time

import data_driven_legged_locomotion.agents.h1_walk.mujoco_mpc.services as mujoco_mpc_agent_module
from data_driven_legged_locomotion.agents.h1_walk.mujoco_mpc import H1PolarWalkAgent
from data_driven_legged_locomotion.agents.h1_walk import FNOStateSpace, OfflineAR2Service
from data_driven_legged_locomotion.agents.h1_walk import MujocoMPCServiceV2 as MujocoMPCService
from data_driven_legged_locomotion.common import MujocoEnvironment, ServiceSet, GreedyMaxEntropyCrowdsouring
from data_driven_legged_locomotion.maps.h1_walk import Map, Cylinder, SlidingWall, Table, Wall, Door, Lamp, Shelf, TV, Couch, Bookshelf, living_room_obstacles
from data_driven_legged_locomotion.tasks.h1_walk import H1WalkEnvironment, H1WalkMapEnvironment, H1TrackCost, h1_quadratic_objective as h1_walk_cost
from data_driven_legged_locomotion.utils import CsvFormatter, GlobalPlanner
from data_driven_legged_locomotion.utils.paths import MA_filter
from data_driven_legged_locomotion.utils.quaternions import yaw_to_quat

from data_driven_legged_locomotion.agents.tdmpc_service import TDMPCService

# Experiment info+
current_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
experiment_name = f"h1_walk_{current_datetime}"
experiment_folder = pathlib.Path(__file__).parent.parent / "experiments" / experiment_name
if not experiment_folder.exists():
    experiment_folder.mkdir(parents=True)

# Logging
logger = logging.getLogger(__name__)
fileHandler = logging.FileHandler(experiment_folder / "experiment_data.csv")
fileHandler.setFormatter(CsvFormatter())
logger.addHandler(fileHandler)

XX1 = [[],[]]
XX2 = [[],[]]
PP = []

# Environment
obstacle_list = []
starting_pos = np.array([2.0, 1.0])
starting_quat = yaw_to_quat(np.pi)
ending_pos = np.array([7.0, 4.0])
# obstacle_list.append(Door(np.array([5.0, 5.0]), quat=yaw_to_quat(0.0), shift_yaw=np.pi))
# obstacle_list.append(Door(np.array([5.0, 5.0]), quat=yaw_to_quat(np.pi), shift_yaw=-np.pi))
#obstacle_list.append(Bookshelf(np.array([3.0, 3.0])))
# obstacle_list.append(Wall(start_pos=np.array([0.0, 0.0]), end_pos=np.array([0.0, 10.0])))
# obstacle_list.append(Wall(start_pos=np.array([6.0, 0.0]), end_pos=np.array([6.0, 10.0])))
# obstacle_list.append(Wall(start_pos=np.array([0.0, 0.0]), end_pos=np.array([6.0, 0.0])))
# obstacle_list.append(Wall(start_pos=np.array([0.0, 10.0]), end_pos=np.array([6.0, 10.0])))
# obstacle_list.append(SlidingWall(np.array([3.33, 0.0]), np.array([3.33, 3.33]), np.array([0.0, 3.33])))
# obstacle_list.append(SlidingWall(np.array([6.66, 0.0]), np.array([6.66, 3.33]), np.array([0.0, 3.33])))
# obstacle_list.append(SlidingWall(np.array([0.0, 6.66]), np.array([3.33, 6.66]), np.array([3.33, 0.0])))
map = Map(obstacles=living_room_obstacles)
env = H1WalkMapEnvironment(map, starting_pos=starting_pos, starting_quat=starting_quat)
model = env.model
global_planner = GlobalPlanner(map, starting_pos, ending_pos)
global_planner.plot_cost()
path = global_planner.get_path()
path = MA_filter(path)
cost = H1TrackCost(env, path)

# Video
video_fps = 60
video_resolution = (720, 1280)
frame_count = 0

video_path = experiment_folder / f"h1_walk_{current_datetime}.mp4"
if not video_path.parent.exists():
    video_path.parent.mkdir()
renderer = mujoco.Renderer(model, height=video_resolution[0], width=video_resolution[1])

# Agent
agent = H1PolarWalkAgent(env)

# Crowdsourcing
ss = FNOStateSpace()
services = ServiceSet(ss)
current_dir = pathlib.Path(mujoco_mpc_agent_module.__file__).parent
models_file = current_dir / "models_ols_fno.pkl"
service_forward = OfflineAR2Service(ss, models_file, "FORWARD")
service_left = OfflineAR2Service(ss, models_file, "LEFT")
service_right = OfflineAR2Service(ss, models_file, "RIGHT")
services.addService(service_forward)
services.addService(service_left)
services.addService(service_right)
crowdsourcing = GreedyMaxEntropyCrowdsouring(ss, services, cost)

#fno
last_fno = np.zeros((2,3))
qpos_old = env.data.qpos[:7][:]
model_timestep = 0.02
downsample_factor = int(model_timestep / 0.002)
i = 0

log_header = ["Time", "State"]
log_header = log_header + [f"Service_{i}_NextState" for i,_ in enumerate(services.services)]
log_header = log_header + ["Service_Index", "Control"]
logger.log(logging.INFO, log_header)
log_row = []

def get_control(env):
  x = env.get_state()
  log_row.append(list(x))
  q, dot_q = env.get_state(split=True)
  x_index = ss.toIndex(x)
  init_time = time.time()
  crowdsourcing.initialize(x, time=env.time)
  for i in range(2):
    next_state = crowdsourcing._behaviors.behaviors[i].getAtTime(0).pf.mean
    log_row.append(list(next_state))
    XX1[i].append(next_state[0])
    XX2[i].append(next_state[1])
    if len(XX1[i]) > 100:
      XX1[i].pop(0)
      XX2[i].pop(0)
  init_time = time.time() - init_time
  print(f"[DEBUG] Initialization time: {init_time}")
  crowdsourcing_time = time.time()
  service_list, behavior = crowdsourcing.run()
  crowdsourcing_time = time.time() - crowdsourcing_time
  print(f"[DEBUG] Total crowdsourcing time: {crowdsourcing_time}")
  service_index = service_list[0]
  log_row.append(service_index)
  PP.append(service_index)
  if len(PP) > 100:
    PP.pop(0)
  u = services.services[service_index].last_u
  return u

def get_control2(env):
  crowdsourcing.initialize(last_fno, time=env.time)
  service_list, behavior = crowdsourcing.run()
  service_index = service_list[0]
  if service_index == 0:
    command = "FORWARD"
  elif service_index == 1:
    command = "LEFT"
  elif service_index == 2:
    command = "RIGHT"
  print(f"[DEBUG] Command {command}")
  agent.set_command(command)
  u = agent.get_control_action()
  return u
  

def plot_sequence():
  plt.figure(1)
  plt.clf()
  plt.stem(PP)
  plt.pause(0.0001)
  plt.figure(2)
  plt.clf()
  ax = plt.gca()
  sigma = 0.01
  plt.scatter(XX1[0], XX2[0], c='r')
  plt.scatter(XX1[1], XX2[1], c='b')
  for i in range(len(XX1[0])):
    circle1 = plt.Circle((XX1[0][i], XX2[0][i]), sigma*2, color='r', fill=False)
    circle2 = plt.Circle((XX1[1][i], XX2[1][i]), sigma*2, color='b', fill=False)
    ax.add_patch(circle1)
    ax.add_patch(circle2)
  plt.pause(0.0001)

path_updated = False

with env.launch_passive_viewer() as viewer:
  with media.VideoWriter(video_path, fps=video_fps, shape=video_resolution) as video:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running():
      log_row = []
      log_row.append(env.time)
      step_start = time.time()

      # Step the simulation forward.
      control_time = time.time()
      u = get_control2(env)
      env.data.mocap_pos = agent.data.mocap_pos # This is needed to visualize the mocap in the viewer
      #log_row.append(list(u))
      control_time = time.time() - control_time
      env.step(u)

      # Pick up changes to the physics state, apply perturbations, update options from GUI.
      viewer.sync()
      
      # Update fno
      if i % downsample_factor == 0:
        last_fno[1] = last_fno[0]
        last_fno[0] = H1WalkEnvironment.get_fno_from_delta(env.data.qpos, qpos_old, model_timestep)
        qpos_old = env.data.qpos[:7][:]
      if env.data.qpos[0] > 2.75:
        env.trigger()
      if np.all([obs.transition_end for obs in map.dynamic_obstacles()]) and not path_updated:
        print("Updating path")
        global_planner.update_start_pos(env.data.qpos[:2])
        path = global_planner.get_path()
        path = MA_filter(path)
        cost = H1TrackCost(env, path)
        crowdsourcing = GreedyMaxEntropyCrowdsouring(ss, services, cost)
        global_planner.plot_cost()
        path_updated = True  
      
      # Render video
      if frame_count < env.time * video_fps:
          renderer.update_scene(env.data, camera="pelvis")
          pixels = renderer.render()
          video.add_image(pixels)
          frame_count += 1
          
      # Log the data
      #logger.log(logging.DEBUG, log_row)
      
      # Rudimentary time keeping, will drift relative to wall clock.
      time_until_next_step = env.timestep - (time.time() - step_start)
      if time_until_next_step > 0:
        time.sleep(time_until_next_step)