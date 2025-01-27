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
Next, We define the cost feature set as,

![equation](https://latex.codecogs.com/png.image?\small&space;\dpi{150}\textbf{h}(\textbf{x}_{k})=[1-\exp(-(\cos(\theta_{k})-1)^{2}),1-\exp(-\omega_{k}^{2})].)

Using the collected data we solve a convex optimisation problem for cost estimation. The cost is reconstructed using the obtained weights.
In the last section of the code we generate the plots in middle panel of Figure 2 of the manuscript.

### IHMCE_MaxEnt_Comparison
- Infinite Horizon Maximum Causal Entropy IRL:
The first part of the code implements the maximum discounted causal Entropy (MDCE) Algorithm of the article 'Infinite Horizon Maximum Causal Entropy Inverse Reinforcement Learning' on the discrete case pendulum. We define the class for soft-value iteration with 0.9 as the discount factor. We define the cost feature set $\textbf{h}(\textbf{x}_{k})$ as defined above. We define the function to calculate the expected feature value of the policy with actual and the policy with estimated cost. To compute the expected feature value for the estimated cost we utilise Monte-Carlo sampling with a sample size of 100. We use the expected feature value functions to optimise the corresponding feature weights using gradient descent.

- Maximum Causal Entropy IRL:
The Second part of the code implements the Algorithm of the article 'Maximum Entropy Inverse Reinforcement Learning' on the discrete case pendulum. We replace the backward pass of the MaxEnt algorithm with value iteration and define the class for value iteration with 0.9 as the discount factor. We define the cost feature set $\textbf{h}(\textbf{x}_{k})$ as defined above. We define the function to calculate the expected feature value of the policy with actual and the policy with estimated cost. To compute the forward pass and obtain expected feature value for estimated. We use the expected feature value of the expert policy and learner policy to optimise the corresponding feature weights using gradient descent.


Note: We implemented   
$\textbf{Experiment settings:}$

- Initial learning rate: 1.0, we use an exponential decay function to update the learning rate after each iteration
- Discount factor for soft-value iteration: 0.9
- Initial weights $\textbf{w}$ are sampled from uniform distribution with support on $[-100,100]$.
- Gradient descent stopping threshold:= 0.001
