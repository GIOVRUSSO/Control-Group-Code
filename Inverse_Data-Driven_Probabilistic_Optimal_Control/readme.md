# ON CONVEX DATA-DRIVEN INVERSE OPTIMAL CONTROL FOR NONLINEAR, NON-STATIONARY AND STOCHASTIC SYSTEMS
### Introduction
This repository collects the supporting code for the manuscript **ON CONVEX DATA-DRIVEN INVERSE OPTIMAL CONTROL FOR NONLINEAR, NON-STATIONARY AND STOCHASTIC SYSTEMS** ([see this link](https://arxiv.org/abs/2306.13928) for the preprint). The manuscript discusses an inverse control problem with a finite time horizon. The goal is to deduce the cost that influences an agent's actions from observations, even when the cost is possibly non-convex and non-stationary. We propose a solution by solving a convex optimization problem that estimates the cost, even in cases where the cost is not convex and the dynamics are nonlinear, non-stationary, and stochastic. Our approach is based on the maximum likelihood principle. We demonstrate the effectiveness of our approach through both computer simulations and experiments using physical hardware.

### Contents
The following list of directories can be found in the repository, which reproduce the simulation and experimental results given in the manuscript.
- pendulum-running-example:
  - This folder contains the code files required for replicating the results of pendulum running example given in the manuscript.
- robotarium:
  - This folder contains the code files required for replicating the results of the robot routing example given in the manuscript. 

### Results
We present the simulation and experimental results given in the manuscript ([see this link](https://arxiv.org/abs/2306.13928))
*Pendulum Example: discrete case*
![pendulumjointci1m06l](https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/2984b2ce-6b53-4627-a838-e368a1b55124)

Target pendulum angular position and corresponding control input.  Results obtained when pfs are discrete and estimated via the histogram filter.  Panels obtained from $20$ simulations and bold lines represent the mean and the shaded region is confidence interval corresponding to the standard deviation(same for all pendulum simulation plots).


![pendulumjointestimatedci1m06l](https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/56d59a10-5ea0-4f10-b8ca-26cb157c2990)

Angular position and corresponding control input, when the cost is estimated using Algorithm 2 given [here](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/Paper_preprint.pdf). The pfs are considered discrete and estimated using histogram filter.

*Pendulum Example: Continous case*

![GPpendulumjointci1m06l](https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/ce5191e2-11ca-4ae7-9f52-681c29b07ce7)
Pendulum angular position and corresponding control input. Results obtained when pfs are estimated via Gaussian Processes. 

![GPpendulumjointci1m06lcostestimated](https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/b6994305-c5e5-47c9-b47b-892badb601fb)

Angular position and corresponding control input, when the cost is estimated using Algorithm 2 from [here](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/Paper_preprint.pdf). The pfs are estimated using Guassian processes.

*Robotarium:*
- In-silico results:

![Cost_Heat_Map_Actual](https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/91936520-70f7-48c9-b9e2-543bc7fa9e0c)

The cost used for the forward control problem.  Note that the cost increases in correspondence of the obstacles (in red in the right panels)  and its minimum is attained at the goal destination

![ALT Text](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/robotarium/Training_Trajectories.jpg)

Robot trajectories starting from different initial conditions depicted as $(\star)$ when the policy from Algorithm 1 from [here](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/Paper_preprint.pdf)) is used.

![Cost_Heat_Map_estimated](https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/5113f33c-7e94-4c2b-b2e3-5dee856f44f2)

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
Author of the code and simulations: *Hozefa Jesawada* (jesawada@unisannio.it)
