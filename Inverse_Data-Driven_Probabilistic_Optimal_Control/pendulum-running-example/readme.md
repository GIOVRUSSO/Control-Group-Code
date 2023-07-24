# Stabilizing a pendulum to unstable equilibrium
This folder contains all the necessary code required to replicate the results of the running pendulum example of the manuscript ([see this link](https://arxiv.org/abs/2306.13928) for the preprint).

### Contents 
The following files are contained in this folder,

- Code Files:
  - ###Pendulum Binning:
      Code for binning/dicretising the pendulum environment space, and obtaining state transistion pmfs for target and reference pendulum.
  - Pendulum GP Modelling: Code for obtaining Gaussian process (GP) models of the reference and target pendulum. 
  - Pendulum IOC Discrete: This code implememts the discrete case of the running example of the manuscript.
  - Pendulum IOC Continuous: This code implememts the continuos case of the running example of the manuscript.
- Model and Data Binaries:
  - Discrete_Pendulum_model.zip: this file stores the discrete pmfs for state transition for the reference and target pendulum.
  - Target_GP_Model.dump: this file stores the GP model for the target pendulum.
  - Reference_GP_Model.dumpp: this file stores the GP model for the reference pendulum.
  - State_H1.npy, State_H2.npy and Control_H.npy: this files store the trajectory data of the pendulum angluar position, angular velocity, and control input respectively.  

### Pendulum Binning

To discretise the state space and obtain discrete pmfs for state transition, we leverage the code provided in the _Probabilistic design of optimal sequential decision-making algorithms in learning and control_ ([see this link](https://arxiv.org/abs/2201.05212)). For more detailed explanation see [link](https://github.com/GIOVRUSSO/Control-Group-Code/tree/master/Decision-making). We export the discrete transition pnfs to *Discrete_Pendulum_model.zip*.

### Pendulum GP Modelling 
This file produces Gaussian process (GP) models of the system dynanics. First, we build a dataset of 3000 state-input pairs by randomly intialising the system and apllying applying inputs. Next, we choose the kernel function and initialise it's parameters. We split the dataset into training and testing batches. After training the GP model we test the model's accuracy on the test set. We export the model binaries to *Target_GP_Model.dump* and *Reference_GP_Model.dump*.

### Pendulum IOC Discrete
- Solving Forward Problem:
This code implememts the discrete case of the running example of the manuscript. It takes Dicrete pmfs of the target and reference pendulum as input. We also define model predictive control (MPC) scheme for reference pendulum with mass =0.5kg. First part of the code computes the policy given the actual state cost. We simulate the pendulum stablisation task by applying the inputs sampled from the policy. We conduct 20 simulations and collect the state-input trajectory data. The data files are saved as *State_H1.npy, State_H2.npy* and *Control_H.npy* and can be used to replicate the results given in the manuscript. 

- Solving Inverse Problem:
Next, we define the feature set as given in the manuscript. Using the collected data we solve a convex optimisation problem for cost estimation. The cost is reconstructed using the obtained weights.
In the last section of the code we generate the plots in left panel of Figure 1 and 2 of the manuscript.


#### Pendulum IOC Continuous 

This code implememts the continuous case of the running example of the manuscript. It takes GP models of the target and reference pendulum as input. To compute the integrals over the Gaussian distribution obtained from GP modelling, we use monte carlo sampling. Specifically, we calculate the average over 500 samples from the distribution. The rest of the code structure remains the same as the discrete case. In the last section of the code we generate the plots in right panel of Figure 1 and 2 of the manuscript.
