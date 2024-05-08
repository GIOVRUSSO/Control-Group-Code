# Benchmarking Algorithm 2
This folder contains all the necessary code required to replicate the results of Benchmarking Algorithm 2 of the manuscript ([see this link](https://arxiv.org/abs/2306.13928) for the preprint).

### Contents 
The following files are contained in this folder,

- Code Files:
  - *Pendulum IOC Benchmarking*: This code implements the discrete case of the running example of the manuscript with the new cost feature vector as defined in Section 3.3 of the manuscript.
  - *IHMCE_MaxEnt_Pendulum*: This code implements the Algorithms from the articles titled 'Maximum Entropy Inverse Reinforcement Learning' and 'Infinite Horizon Maximum Causal Entropy Inverse Reinforcement Learning' on the discrete case pendulum.
 
### Pendulum IOC Benchmarking
- Solving Forward Problem:
This code implememts the discrete case of the running example of the manuscript. It takes Dicrete pmfs of the target and reference pendulum as input. We also define model predictive control (MPC) scheme for reference pendulum with mass =0.5kg. First part of the code computes the policy given the actual state cost. We simulate the pendulum stablisation task by applying the inputs sampled from the policy. We conduct 20 simulations and collect the state-input trajectory data.  

- Solving Inverse Problem:
Next, We define the cost feature set as given in the Section 3.3 of the manuscript. Using the collected data we solve a convex optimisation problem for cost estimation. The cost is reconstructed using the obtained weights.
In the last section of the code we generate the plots in middle panel of Figure 2 of the manuscript.

### IHMCE_MaxEnt_Comparison
-Infinite Horizon Maximum Causal Entropy IRL:
The first part of the code implements the maximum discounted causal Entropy (MDCE) Algorithm of the article 'Infinite Horizon Maximum Causal Entropy Inverse Reinforcement Learning'. We define the class for soft value iteration. We define the function to calculate the expected feature value of the policy with actual and the policy with estimated cost. We use the expected value feature functions to perform the optimisation using the gradient descent.

  

