from pathlib import Path
import numpy as np

from data_driven_legged_locomotion.common import StateSpace, MujocoEnvironment, DiscreteStateSpace
from data_driven_legged_locomotion.agents.pendulum import SwingUpService

class PendulumEnvironment(MujocoEnvironment):
    def __init__(self):
        model_path = Path(__file__).parent / 'simple_pendulum.xml'
        ss = DiscreteStateSpace(2, np.array([[-2*np.pi, 2*np.pi], [-10, 10]]), [np.pi/100, 20/2000])
        super().__init__(ss, model_path)
        
def energy_based_cost(states,k):
  g = 9.81
  m = 5.5
  l = 0.5
  if len(states.shape) == 1:
    states = np.expand_dims(states, axis=0)
  #states is now n_samples x n_states
  #ref = np.array([np.pi, 0])
  #cost = np.sum((states-ref)**2, axis=1)
  E_actual = 0.5 * m * l**2 * states[:,1]**2 - m * g * l * np.cos(states[:,0])
  E_desired = m * g * l
  pos_actual = states[:,0]
  pos_desired = np.pi
  cost = (E_actual - E_desired)**2 + 100*(pos_actual - pos_desired)**2
  # print(f"energy error: {np.mean((E_actual - E_desired)**2)}")
  # print(f"position error: {np.mean(100*(pos_actual - pos_desired)**2)}")
  cost = np.squeeze(cost)
  return cost

# A2 = [[0,1],[0,0]]
# B2 = [[0],[1]]
# C2 = [[1,0]]
# D2 = 0
# sys2 = control.ss(A2, B2, C2, D2)
# sys2 = control.c2d(sys2, 0.002)
# Q2 = [[1, 0], [0, 1]]
# R2 = [0.1]
# K2, S2, E2 = control.dlqr(sys2, Q2, R2)

# HH = []

# ss = StateSpace(2, np.array([[-2*np.pi, 2*np.pi], [-10, 10]]), [np.pi/100, 20/2000])

# global_model_path = Path(__file__).parent / 'simple_pendulum.xml'
# global_model_path = str(global_model_path)
# global_model = mujoco.MjModel.from_xml_path(global_model_path)
# global_data = mujoco.MjData(global_model)

def cost(states,k):
  g = 9.81
  m = 5.5
  l = 0.5
  if len(states.shape) == 1:
    states = np.expand_dims(states, axis=0)
  #states is now n_samples x n_states
  #ref = np.array([np.pi, 0])
  #cost = np.sum((states-ref)**2, axis=1)
  E_actual = 0.5 * m * l**2 * states[:,1]**2 - m * g * l * np.cos(states[:,0])
  E_desired = m * g * l
  pos_actual = states[:,0]
  pos_desired = np.pi
  cost = (E_actual - E_desired)**2 + 100*(pos_actual - pos_desired)**2
  print(f"energy error: {np.mean((E_actual - E_desired)**2)}")
  print(f"position error: {np.mean(100*(pos_actual - pos_desired)**2)}")
  cost = np.squeeze(cost)
  return cost

# def energy_shaping_controller(x):
#   (q, dot_q) = x
#   g = 9.81
#   m = 5.5
#   l = 0.5
#   b = 3.5
#   k = 10
#   E_actual = 0.5 * m * l**2 * dot_q**2 - m * g * l * np.cos(q)
#   E_desired = m * g * l
#   tilde_E = E_actual - E_desired
#   u = - k * dot_q * tilde_E
#   return u

# def lqr_controller(x):
#   (q, dot_q) = x
#   delta_q = q - np.pi
#   delta_dot_q = dot_q
#   delta_x = np.array([delta_q, delta_dot_q])
#   u = - np.dot(K, delta_x)
#   return u

def lqr_controller_2(x,r):
  (q, dot_q) = x
  delta_q = q - r
  delta_dot_q = dot_q
  delta_x = np.array([delta_q, delta_dot_q])
  u = - np.dot(K2, delta_x) + K2[0,0] * r
  return u

def linearize_plant(x,v):
  # Produces the equivalent plant ddot_q = v
  # Pendulum dynamics: ddot_q = -g/l sin(q) - b/(m*l^2) dot_q + 1/(m*l^2) u
  (q, dot_q) = x
  g = 9.81
  m = 5.5
  l = 0.5
  b = 3.5
  u = m * l**2 * (v + g/l * np.sin(q) + b/(m*l**2) * dot_q)
  return u

def feedback_linearization_controller(x,r):
  v = - np.dot(K2, x - np.array([r,0])) + K2[0,0] * r
  u = linearize_plant(x,v)
  return u
  
# services = ServiceSet(ss)
# SwingUpService = SwingUpService(ss, global_model)
# lqrService = LQRService(ss, global_model)
# services.addService(SwingUpService)
# services.addService(lqrService)
# crowdsourcing = MaxEntropyCrowdsouring(ss, services, cost)

def fsm_controller(x):
  (q, dot_q) = x
  # If the pendulum is near the downward equilibrium position give it a small push
  if q > -0.001 and q < 0.001 and dot_q < 0.00001:
    return 0.1
  # If the pendulum is near the upright equilibrium position use the LQR controller to stabilize it
  if (q > 2.96 or q < -2.96) and dot_q < 0.2:
    u = lqrService.policy(x)
  else: # Otherwise use the energy shaping controller to swing up the pendulum
    u = SwingUpService.policy(x)
  return u

def control_callback(model, data):
  q = data.joint('hinge').qpos[0]
  dot_q = data.joint('hinge').qvel[0]
  x = np.array([q, dot_q])
  x_index = ss.toIndex(x)
  crowdsourcing.initialize(x)
  service_list, behavior = crowdsourcing.run()
  service_index = service_list[0]
  HH.append(service_index)
  if service_index == 0:
    u = SwingUpService.policy(x)
  elif service_index == 1:
    u = lqrService.policy(x)
  # u = fsm_controller((q, dot_q))
  # data.qfrc_applied[0] = data.qfrc_bias[0]
  # u = lqr_controller_2(x, np.pi)
  #u = fsm_controller((q, dot_q))
  data.actuator('torque').ctrl[0] = u
  
def get_control(env):
  x = env.get_state()
  q, dot_q = env.get_state(split=True)
  x_index = ss.toIndex(x)
  crowdsourcing.initialize(x)
  service_list, behavior = crowdsourcing.run()
  service_index = service_list[0]
  HH.append(service_index)
  if service_index == 0:
    u = SwingUpService.policy(x)
  elif service_index == 1:
    u = lqrService.policy(x)
  # u = fsm_controller((q, dot_q))
  # data.qfrc_applied[0] = data.qfrc_bias[0]
  # u = lqr_controller_2(x, np.pi)
  #u = fsm_controller((q, dot_q))
  return u

# model_path = Path(__file__).parent / 'simple_pendulum.xml'
# env = MujocoEnvironment(ss, model_path)

# with env.launch_passive_viewer() as viewer:
#   # Close the viewer automatically after 30 wall-seconds.
#   start = time.time()
#   while viewer.is_running():
#     step_start = time.time()

#     #control_callback(model,data)
#     # Step the simulation forward.
#     #mujoco.mj_step(model, data)
    
#     u = get_control(env)
#     env.step(u)

#     # Pick up changes to the physics state, apply perturbations, update options from GUI.
#     viewer.sync()

#     # Rudimentary time keeping, will drift relative to wall clock.
#     time_until_next_step = env.timestep - (time.time() - step_start)
#     if time_until_next_step > 0:
#       time.sleep(time_until_next_step)