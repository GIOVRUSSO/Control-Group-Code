# ON CONVEX DATA-DRIVEN INVERSE OPTIMAL CONTROL FOR NONLINEAR, NON-STATIONARY AND STOCHASTIC SYSTEMS
### Introduction
This repository collects the supporting code for the manuscript **ON CONVEX DATA-DRIVEN INVERSE OPTIMAL CONTROL FOR NONLINEAR, NON-STATIONARY AND STOCHASTIC SYSTEMS** ([see this link]()). The manuscript discusses an inverse control problem with a finite time horizon. The goal is to deduce the cost that influences an agent's actions from observations, even when the cost is possibly non-convex and non-stationary. We propose a solution by solving a convex optimization problem that estimates the cost, even in cases where the cost is not convex and the dynamics are nonlinear, non-stationary, and stochastic. Our approach is based on the maximum likelihood principle. We demonstrate the effectiveness of our approach through both computer simulations and experiments using physical hardware.

### Contents
The following list of files can be found in the repository, which reproduce the parameters/figures in the manuscript.
- pendulum-running-example
- robotarium

### Results
Pendulum Example: discrete case

![Alt Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/pendulum-running-example/pendulum_joint_ci_1m_0.6l.jpeg)

![Alt Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/pendulum-running-example/pendulum_joint_estimated_ci_1m_0.6l.jpg)

Pendulum Example: Continous case

![ALT Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/pendulum-running-example/GP_pendulum_joint_ci_1m_0.6l.jpeg)

![ALT Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/pendulum-running-example/GP_pendulum_joint_ci_1m_0.6l_cost_estimated.jpeg)

Robotarium:
- In-silico results:

![ALT Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/robotarium/Cost_Heat_Map%20(1).jpg)

![ALT Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/robotarium/Training_Trajectories.jpg)

![alt text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/robotarium/Cost_Heat_Map_estimated.jpg)

![Alt text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/robotarium/Test_Trajectories.jpg)




- Experimental results:

https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/.github/images/robotarium_video.mp4

