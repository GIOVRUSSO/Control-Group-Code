# Benchmarking Algorithm 2
This folder contains all the necessary code required to replicate the results of Benchmarking Algorithm 2 of the manuscript ([see this link](https://arxiv.org/abs/2306.13928) for the preprint).

### Contents 
The following files are contained in this folder,

- Code Files:
  - *Pendulum IOC Benchmarking*: This code implements the discrete case of the running example of the manuscript with the new cost feature vector as defined in Section 3.3 of the manuscript.
  - *IHMCE_MAXENT_Pendulum*: This code implements the Algorithms from the articles titled 'Maximum Entropy Inverse Reinforcement Learning' and 'Infinite Horizon Maximum Causal Entropy Inverse Reinforcement Learning'.
- Model and Data Binaries:
  - *Discrete_Pendulum_model.zip*: this file stores the discrete pmfs for state transition for the reference and target pendulum.
  - *Target_GP_Model.dump*: this file stores the GP model for the target pendulum.
  - *Reference_GP_Model.dump*: this file stores the GP model for the reference pendulum.
  - *State_H1.npy, State_H2.npy* and *Control_H.npy*: this files store the trajectory data of the pendulum angluar position, angular velocity, and control input respectively.  

