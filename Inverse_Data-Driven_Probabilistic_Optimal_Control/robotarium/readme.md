# Robotarium 
### Prerequisites
To the run code, the first step is to download and install the [robotarium python simulator package](https://github.com/robotarium/robotarium_python_simulator). Next, copy the files named *robot_routing_IOC.ipynb*, *robot_routing_simulate.py*, and *Weights.npy* to the same folder.
### Contents 
The Robotarium folder contains following files:
- robot_routing_IOC.ipynb
- robot_routing_simulate.py

### robot_routing_IOC.ipynb

Given the setup of the experiment in the manuscript, the first part in the code solves the forward problem/task of robot routing while avoiding obstacles using the control policy computed by Algorithm 1 of the manuscript. Multiple state and control input trajectories of robot performing the task are obtained and saved in the *State_Data.npy* and *Input_Data.npy*. Then, the second part of the code uses these data files to estimate the cost of the agent using Algorithm 2 of the manuscript. We define a function which computes the features as defined in the manuscript. Next, we obtain the *weights.npy* by solving the inverse problem. We use the weights to formulate the estimated cost and test the effectiveness of the estimated cost by performing the robot routing cost while avoiding the obstacles. The plots in the figure 3 of the manuscripts can be obtained by the last section of the code.

### robot_routing_simulate.py

This file simulated the robot performing the obstacle avoidance task. Unlike *si_go_to_point.ipynb*, we can visualize the agent performing the task using this code. We provide *Weights.npy* as an input and compute the policy of the agent. Apply the input sampled from the policy and continue this process till the task of reaching the goal is completed. We can also obtain state and input trajectories data as a .npy file. 

Apart from simulation, this code file can also be used to perform hardware experiments. Just upload the *robot_routing_simulate.py* and *Weights.npy* files to the Robotarium portal and you can obtain the video of the robot performing the task.
