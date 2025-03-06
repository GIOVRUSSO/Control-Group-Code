# Belief Update Benchmark
This folder contains all the necessary code required to implement the benchmark algorithm from the article titled 'Infinite Horizon Maximum Causal Entropy Inverse Reinforcement Learning'.

### Contents 
The following files are contained in this folder,

- Code Files:
  - *IHMCE_MaxEnt_Robotarium.ipynb*: This code implements the Algorithms from the article titled 'Infinite Horizon Maximum Causal Entropy Inverse Reinforcement Learning' on the discrete Robotarium.
  - *optimizer.py*: This file implements the gradient ascent-based optimization.

### IHMCE_MaxEnt_Comparison
- Infinite Horizon Maximum Causal Entropy IRL:

The code implements the **Maximum Discounted Causal Entropy (MDCE) Algorithm** from the article *"Infinite Horizon Maximum Causal Entropy Inverse Reinforcement Learning"* in a discrete-state Robotarium environment. The state space is discretized into a **50×50 grid**, and a **soft-value iteration** class is defined using a **0.9 discount factor**. The cost feature set follows the definitions in the manuscript. To evaluate policy performance, the code computes the **expected feature values** for both the actual policy and the policy derived from the estimated cost. The expected feature values for the estimated cost are obtained via **Monte Carlo sampling** with a sample size of **100**. These values are then leveraged to optimize feature weights using **gradient ascent**, refining the learned cost representation.

![Cost_Heat_Map_IHMCE](https://github.com/user-attachments/assets/8fa6d258-5acc-4381-8d64-7d4fd08e0879)

Figure. Cost reconstructed by IHMCE algorithm. 

Note: We implemented the benchmarking algorithm (IHMCE) as efficiently as possible. 

$\textbf{Experiment settings:}$
- Initial learning rate: 1.0 (we use an exponential decay function to update the learning rate after each iteration)
- Discount factor for soft-value iteration: 0.9
- Initial weights $\textbf{w}$ are sampled from a uniform distribution with support on $[-100,100]$ 
- Gradient ascent stopping threshold:= 0.001
