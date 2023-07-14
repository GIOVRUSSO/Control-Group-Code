# Robotarium 
### Prerequisites
To the run code, the first step is to download and install the [robotarium python simulator package](https://github.com/robotarium/robotarium_python_simulator). Next, copy the files named *robot_routing_IOC.ipynb*, *robot_routing_simulate.py*, and *Weights_Obtained.npy* to the same folder.
### Contents 
The Robotarium folder contains the following files:
- robot_routing_IOC.ipynb
- robot_routing_simulate.py

### robot_routing_IOC.ipynb

Given the setup of the experiment in the manuscript, the first part of the code solves the forward problem/task of robot routing while avoiding obstacles using the control policy computed by Algorithm 1 of the manuscript. Multiple state and control input trajectories of a robot performing the task are obtained and saved in the *State_Data.npy* and *Input_Data.npy*. Then, the second part of the code uses these data files to estimate the cost of the agent using Algorithm 2 of the manuscript. 

We define a function that forms the feature vector. We used a 16-dimensional features vector, with the first feature being equal to, 

![equation1](https://latex.codecogs.com/svg.image?%5Cbg%7Bwhite%7D(%5Cmathbf%7Bx%7D_%7Bk%7D-%5Cmathbf%7Bx%7D_%7Bd%7D)%5E%7B2%7D),

and with the other features being Gaussians of the form,

![equation](https://latex.codecogs.com/png.image?\large&space;\dpi{110}\bg{white}g_{i}(\mathbf{x}_{k}):=\frac{1}{\sqrt{{(2\pi)^{2}\det(\mathbf{\Sigma}_o)}}}\exp\left(-\frac{1}{2}(\mathbf{x}_{k}-\mathbf{o}_{i})^\top\mathbf{\Sigma}_o^{-1}(\mathbf{x}_{k}-\mathbf{o}_{i})\right)),

but centered around 15 uniformly distributed points in the Robotarium work area.


![feature_point_grid](https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/d50ee3e0-3e3b-4595-b5fc-a3305e843b08)


Next, we obtain the *weights.npy* by solving the inverse problem. We use the weights to formulate the estimated cost and test the effectiveness of the estimated cost by performing the robot routing cost while avoiding obstacles. The plots in Figure 3 of the manuscripts can be obtained from the last section of the code.

Note: To replicate Figure 3 of the manuscript use the *Weights_Obtained.npy*, *State_Data*, and *Input_Data* files.

### robot_routing_simulate.py

This file simulated the robot performing the obstacle avoidance task. Unlike *robot_routing_IOC.ipynb*, we can visualize the agent performing the task using this code. We provide *Weights.npy* as an input and compute the policy of the agent. Apply the input sampled from the policy and continue this process till the task of reaching the goal is completed. We can also obtain state and input trajectory data as a .npy file. 

Apart from simulation, this code file can also be used to perform hardware experiments. Just upload the *robot_routing_simulate.py* and *Weights.npy* files to the Robotarium portal and you can obtain the video of the robot performing the task.
