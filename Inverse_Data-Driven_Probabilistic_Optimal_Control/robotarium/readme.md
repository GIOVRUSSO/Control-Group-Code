# Robotarium 

This folder contains the code required to perform the robot routing experiments as given in the manuscript ([see this link](https://arxiv.org/abs/2306.13928) for the preprint).
### Prerequisites
To the run code, the first step is to download and install the [robotarium python simulator package](https://github.com/robotarium/robotarium_python_simulator). Next, copy the files named *robot_routing_IOC.ipynb*, *robot_routing_simulate.py*, and *Weights_Obtained.npy* to the same folder.
### Contents 
The Robotarium folder contains the following files:

- Code Files:
  - *robot_routing_IOC.ipynb*: The notebook implements the robot routing experiment by solving the forward and inverse using the algorithms given in the manuscript.
  - *robot_routing_simulate.py*: The code file implements the robot routing task and can be used to perform physical experiment on the robotarium platform.
  - *robot_routing_continuous.ipynb*: The notebook implements the robot routing experiment for scenario 1 of the manuscript by solving the routinh problem for the continuous state\action space by using the Gaussian policy derived in equation (11) of the manuscript.
- Binaries:
  - *State_Data.npy*: Stores state trajectory data of the agent performing task with known actual cost.
  - *Input_Data.npy*: Stores control input trajectory data of the agent performing task with known actual cost.
  - *Weights_Obtained.npy*: Stores the weights obtained that can replicate the results in the manuscript.
### robot_routing_IOC.ipynb

- Forward Problem:
Given the setup of the experiment in the manuscript, the first part of the code solves the forward problem/task of robot routing while avoiding obstacles using the control policy computed by Algorithm 1 of the manuscript. Multiple state and control input trajectories of a robot performing the task are obtained and saved in the *State_Data.npy* and *Input_Data.npy*. 

- Inverse Problem:
The second part of the code uses these data files to estimate the cost of the agent using Algorithm 2 of the manuscript. 
We define a function that forms the feature vector. We used a 16-dimensional features vector, with the first feature being, $$(x_{k}-x_{d})^{2}$$  the distance from the desired location of the robot,
and with the other features being Gaussians of the form,


![equation](https://latex.codecogs.com/png.image?\large&space;\dpi{110}\bg{white}g_{i}(\mathbf{x}_{k}):=\frac{1}{\sqrt{{(2\pi)^{2}\det(\mathbf{\Sigma}_o)}}}\exp\left(-\frac{1}{2}(\mathbf{x}_{k}-\mathbf{o}_{i})^\top\mathbf{\Sigma}_o^{-1}(\mathbf{x}_{k}-\mathbf{o}_{i})\right)),

with ![equation](https://latex.codecogs.com/png.image?%5Cdpi%7B110%7D%5Cbg%7Bwhite%7D%5Cmathbf%7B%5CSigma%7D_o=%5Cbegin%7Bbmatrix%7D0.025&0%5C%5C0&0.025%5C%5C%5Cend%7Bbmatrix%7D), centered around 15 uniformly distributed points in the Robotarium work area. 
Next, we obtain the *weights.npy* by solving the inverse problem. The figure below shows the placement of the feature points on the Robotarium work area with corresponding weights value.
![feature_point_grid](https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/d50ee3e0-3e3b-4595-b5fc-a3305e843b08)
We use the weights to formulate the estimated cost and test the effectiveness of the estimated cost by performing the robot routing cost while avoiding obstacles. The plots in Figure 3 of the manuscripts can be obtained from the last section of the code.

Note: To replicate Figure 3 of the manuscript use the *Weights_Obtained.npy*, *State_Data*, and *Input_Data* files.

### robot_routing_simulate.py

This file simulated the robot performing the obstacle avoidance task. Unlike *robot_routing_IOC.ipynb*, we can visualize the agent performing the task using this code. We provide *Weights.npy* as an input and compute the policy of the agent. Apply the input sampled from the policy and continue this process till the task of reaching the goal is completed. We can also obtain state and input trajectory data as a .npy file. 

Apart from simulation, this code file can also be used to perform hardware experiments. Just upload the *robot_routing_simulate.py* and *Weights.npy* files to the Robotarium portal and you can obtain the video of the robot performing the task.

### robot_routing_continuous.ipynb

### robot_routing_IOC.ipynb

- Forward Problem:
Given the setup of scenario 1 of the application example in the manuscript, the first part of the code solves the forward problem/task of robot routing considering a quadratic state cost and using the continuous action space control policy computed by equation (11) of the manuscript. Multiple state and control input trajectories of a robot performing the task are obtained and shown in the top left panel in Figure 4 of the manuscript. 

- Inverse Problem:
The second part of the code uses these data files to estimate the cost of the agent using Algorithm 2 of the manuscript. 
We define a function that forms the feature vector. We used a 15-dimensional features vector, with the feature being, $$(x_{k}-x_{i})^{2}$$  the distance from the location point $x_{i}$ of the robot.
