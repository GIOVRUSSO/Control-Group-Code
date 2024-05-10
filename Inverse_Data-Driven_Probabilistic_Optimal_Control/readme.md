# ON CONVEX DATA-DRIVEN INVERSE OPTIMAL CONTROL FOR NONLINEAR, NON-STATIONARY AND STOCHASTIC SYSTEMS
### Introduction
This repository collects the supporting code for the manuscript **ON CONVEX DATA-DRIVEN INVERSE OPTIMAL CONTROL FOR NONLINEAR, NON-STATIONARY AND STOCHASTIC SYSTEMS** ([see this link](https://arxiv.org/abs/2306.13928) for the preprint). The manuscript discusses an inverse control problem with a finite time horizon. The goal is to deduce the cost that influences an agent's actions from observations, even when the cost is possibly non-convex and non-stationary. We propose a solution by solving a convex optimization problem that estimates the cost, even in cases where the cost is not convex and the dynamics are nonlinear, non-stationary, and stochastic. Our approach is based on the maximum likelihood principle. We demonstrate the effectiveness of our approach through both computer simulations and experiments using physical hardware.

### Contents
The following list of directories can be found in the repository, which reproduces the simulation and experimental results given in the manuscript.
- pendulum-running-example:
  - This folder contains the code files required for replicating the results of the pendulum running example given in the manuscript.
- robotarium:
  - This folder contains the code files required for replicating the results of the robot routing example given in the manuscript.
- IHMCE_MaxEnt_Comparison:
  - This folder contains the code files required for replicating the results of benchmarking Algorithm 2 given in the manuscript.
- grid world example:
  - This folder contains the basic implementation of MaxEnt/IHMCE on a simple grid world environment     

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

*Benchmarking Algorithm 2:*

Algorithm 2 presented in the manuscript ([see this link](https://arxiv.org/abs/2306.13928)) is benchmarked against the Algorithms presented in the articles titled 'Maximum Entropy Inverse Reinforcement Learning' and 'Infinite Horizon Maximum Causal Entropy Inverse Reinforcement Learning'. 

<img width="650" alt="Screenshot 2024-05-08 154526" src="https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/f933c271-4d55-48c6-accc-ae7352b7c4b2">

Figure: Top left: original cost function. In the other panels, the cost reconstructed via: Algorithm 2 (top-right), MaxEnt
(bottom-left) and IHMCE (bottom-right).

*Robotarium:*
- Scenario 1: In-silico results

  <img width="869" alt="Screenshot 2024-05-08 165800" src="https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/fd6fe888-0379-45dd-b579-f51efaf835e0">

Figure. Top-left: robot trajectories starting from different initial positions (⋆) when the policy in (11) - (12) is used (with N = 1). Top-right: the oi’s together with the weights obtained via Algorithm 2. Bottom: reconstructed cost (left) and robot trajectories when Algorithm 1 is used with this cost. The robot starts from initial positions that are different from those in the top panel.
  
- Scenario 2: In-silico results

<img width="779" alt="Screenshot 2024-05-10 165445" src="https://github.com/GIOVRUSSO/Control-Group-Code/assets/62793703/eea03072-9a55-4545-b3dd-632d52a4260c">

Figure. Top-left: cost for the FOC problem. Top-right: robot trajectories when the policy from Algorithm 1 is used (same initial positions and destination of Scenario 1). Bottom panels: cost reconstructed via Algorithm 2 (left) and robot trajectories when Algorithm 1 is used with the estimated cost. Robots start from initial positions that are different from these in the top panel.



- Experimental results:

The following videos show robotarium robot performing the task:
  - When the control policy using Algoritm 1 is applied while knowing the actual cost.


https://github.com/Yakub-Jesawada/Wikipedia/assets/98798839/47b78536-bfd8-49af-9510-6fd19da477d2

  - When the control policy using Algoritm 1 is applied while estimating the cost using Algorithm 2 given in [link](https://github.com/GIOVRUSSO/Control-Group-Code/blob/master/Inverse_Data-Driven_Probabilistic_Optimal_Control/Paper_preprint.pdf).

https://github.com/Yakub-Jesawada/Wikipedia/assets/98798839/1dd524f3-55cf-4940-bd06-dd39ce451534

### Authors and Contributors 
Author of the code and simulations: *Hozefa Jesawada* (jesawada@unisannio.it)
