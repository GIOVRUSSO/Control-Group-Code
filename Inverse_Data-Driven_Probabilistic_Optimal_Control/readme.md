# ON CONVEX DATA-DRIVEN INVERSE OPTIMAL CONTROL FOR NONLINEAR, NON-STATIONARY AND STOCHASTIC SYSTEMS
### Introduction
This repository collects the supporting code for the manuscript **ON CONVEX DATA-DRIVEN INVERSE OPTIMAL CONTROL FOR NONLINEAR, NON-STATIONARY AND STOCHASTIC SYSTEMS** ([see this link](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/Paper_preprint.pdf) for the preprint). The manuscript discusses an inverse control problem with a finite time horizon. The goal is to deduce the cost that influences an agent's actions from observations, even when the cost is possibly non-convex and non-stationary. We propose a solution by solving a convex optimization problem that estimates the cost, even in cases where the cost is not convex and the dynamics are nonlinear, non-stationary, and stochastic. Our approach is based on the maximum likelihood principle. We demonstrate the effectiveness of our approach through both computer simulations and experiments using physical hardware.

### Contents
The following list of directories can be found in the repository, which reproduce the simulation and experimental results given in the manuscript.
- pendulum-running-example
- robotarium

### Results
*Pendulum Example: discrete case*

![Alt Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/pendulum-running-example/pendulum_joint_ci_1m_0.6l.jpeg)

Target pendulum angular position and corresponding control input.  Results obtained when pfs are discrete and estimated via the histogram filter.  Panels obtained from $20$ simulations and bold lines represent the mean and the shaded region is confidence interval corresponding to the standard deviation(same for all pendulum simulation plots).


![Alt Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/pendulum-running-example/pendulum_joint_estimated_ci_1m_0.6l.jpg)

Angular position and corresponding control input, when the cost is estimated using algorithm 2 of [see link](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/Paper_preprint.pdf). The pfs are considered discrete and estimated using histogram filter.

*Pendulum Example: Continous case*

![ALT Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/pendulum-running-example/GP_pendulum_joint_ci_1m_0.6l.jpeg)

Pendulum angular position and corresponding control input. Results obtained when pfs are estimated via Gaussian Processes. 

![ALT Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/pendulum-running-example/GP_pendulum_joint_ci_1m_0.6l_cost_estimated.jpeg)

Angular position and corresponding control input, when the cost is estimated using algorithm 2 of [see link](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/Paper_preprint.pdf). The pfs are estimated using Guassian processes.

*Robotarium:*
- In-silico results:

![ALT Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/robotarium/Cost_Heat_Map%20(1).jpg)

The cost used for the forward control problem.  Note that the cost increases in correspondence of the obstacles (in red in the right panels)  and its minimum is attained at the goal destination

![ALT Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/robotarium/Training_Trajectories.jpg)

Robot trajectories starting from different initial conditions depicted as $(\star)$ when the policy from Algorithm 1 in )[see link](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/Paper_preprint.pdf)) is used.

![alt text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/robotarium/Cost_Heat_Map_estimated.jpg)

Estimated cost using Algorithm 2.


![Alt text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/robotarium/Test_Trajectories.jpg)

Trajectories under algorithm 1 with estimated cost.




- Experimental results:

The following videos show robotarium robot performing the task:
  - When the control policy using Algoritm 1 is applied while knowing the actual cost.


https://github.com/Yakub-Jesawada/Wikipedia/assets/98798839/47b78536-bfd8-49af-9510-6fd19da477d2

  - When the control policy using Algoritm 1 is applied while estimating the cost using Algorithm 2 given in [link](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/Paper_preprint.pdf).

https://github.com/Yakub-Jesawada/Wikipedia/assets/98798839/1dd524f3-55cf-4940-bd06-dd39ce451534

### Authors and Contributors 
Author: *Hozefa Jesawada* (jesawada@unisannio.it)
