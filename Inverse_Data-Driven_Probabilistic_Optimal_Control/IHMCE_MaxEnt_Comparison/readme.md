# Benchmarking Algorithm 2
This folder contains all the necessary code required to replicate the results of Benchmarking Algorithm 2 of the manuscript ([see this link](https://arxiv.org/abs/2306.13928) for the preprint).

### Contents 
The following files are contained in this folder,

- Code Files:
  - *Pendulum IOC Benchmarking*: This code implements the discrete case of the running example of the manuscript with the new cost feature vector as defined in Section 3.3 of the manuscript.
  - *IHMCE_MaxEnt_Pendulum*: This code implements the Algorithms from the articles titled 'Maximum Entropy Inverse Reinforcement Learning' and 'Infinite Horizon Maximum Causal Entropy Inverse Reinforcement Learning' on the discrete case pendulum.
 
### Pendulum IOC Benchmarking
- Solving Forward Problem:
This code implememts the discrete case of the running example of the manuscript. It takes Dicrete pmfs of the target and reference pendulum as input. We also define model predictive control (MPC) scheme for reference pendulum with mass =0.5kg. First part of the code computes the policy given the actual state cost. We simulate the pendulum stablisation task by applying the inputs sampled from the policy. We conduct 20 simulations and collect the state-input trajectory data. The data files are saved as *State_H1.npy, State_H2.npy* and *Control_H.npy* and can be used to replicate the results given in the manuscript. 

- Solving Inverse Problem:
Next, We define the cost feature set as given in the Section 3.3 of the manuscript. Using the collected data we solve a convex optimisation problem for cost estimation. The cost is reconstructed using the obtained weights.
In the last section of the code we generate the plots in middle panel of Figure 2 of the manuscript.

  

