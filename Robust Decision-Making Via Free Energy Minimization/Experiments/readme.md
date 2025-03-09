# Experiments

This folder contains the code required to perform the robot routing experiments as given in the manuscript ([see this link](https://arxiv.org/abs/2306.13928) for the preprint).
### Prerequisites
To run the code, the first step is to download and install the [robotarium python simulator package](https://github.com/robotarium/robotarium_python_simulator).
### Contents 
The **Experiments** folder contains the following files:

- Code Files:
  - *DR_robot_routing_simulate.py*: The code file implements the DR-FREE Algorithm and performs the robot routing task. The code can be submitted to the robotarium platform. 
  - *DR_robot_routing_IOC.ipynb*: The notebook implements the robot routing experiment by solving the forward and inverse using the algorithms given in the manuscript.
  - *GP_Model_Training.ipynb*: The notebook contains code to train GP models.
  - *eta_policy.ipynb*: The notebook evaluates the policy computation of DR-FREE algorithm under varying ambiguity radius.
  - *Robo_Dataset_Generate.py*: The code file generates data for training GP models.
- Binaries:
  - *GP_nominal_1.dump*: Stores GP model for training stage 1.
  - *GP_nominal_2.dump*: Stores GP model for training stage 2.
  - *GP_nominal.dump*: Stores GP model for training stage 3.
  - *Weights_DR.npy*: Stores the weights obtained for the reconstructed cost that can replicate the results in the manuscript.

### DR_robot_routing_simulate.py

The file **DR_robot_routing_simulate.py** implements the simulation environment for the DR-FREE framework on a robot routing task. It loads a pre-trained Gaussian Process model to predict nominal state transitions and defines dynamic models, cost functions, and robust control steps that integrate obstacle avoidance, goal attainment, and environmental uncertainties. Leveraging the Robotarium simulation tools the script simulates robot navigation within a bounded workspace populated with obstacles and boundaries. Throughout the simulation, it records state trajectories and control inputs, which are then saved for further analysis.

- The file also implements an ambiguity-unaware agent, the control algorithm can be switched by commenting out the DR-FREE algorithm at lines 318 and 319, and uncommenting lines 322.

### DR_robot_routing_IOC.ipynb

- DR-FREE Algorithm:
The first part of the code implements the DR-FREE algorithm given in the manuscript and generates robot trajectory data.

- Belief update:
The second part of the code uses these data files to estimate the cost of the agent using the belief update algorithm of the manuscript. 
We define a function that forms the feature vector.  Next, we obtain the *Weights_DR.npy* by solving the convex belief update problem. The figure below shows the placement of the feature points on the Robotarium work area with corresponding weight values.
![feature_point_grid](https://github.com/user-attachments/assets/f749acb2-1d2f-4234-8e71-0b165b21e832)
We use the weights to formulate the estimated cost and test the effectiveness of the estimated cost by performing the robot routing cost while avoiding obstacles.

### eta_policy.ipynb

The notebook shows how DR-FREE policy changes as a function of the ambiguity radius $(\eta(x_{k-1},u_{k}))$
![Screenshot 2025-03-09 144722](https://github.com/user-attachments/assets/b6fce2fe-0e57-4287-b5cd-5ff86f0419b9)

Figure. By increasing the radius of ambiguity $(\eta(x_{k-1},u_{k}))$, the DR-FREE policy (left) becomes $(p^{(u)^{\star}}_{k}\propto q^{(u)}_{k}\eta(x_{k-1},u_{k}))$ (right).

### GP_Model_Training.ipynb

The notebook implements GP training by leveraging the *scikit-learn* library.
