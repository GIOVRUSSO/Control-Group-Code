posi# Stabilizing a pendulum to unstable equilibrium
This folder contains all the necessary code required to replicate the results of the running pendulum example of the manuscript.

### Contents 
The following files are contained in this folder,
- Pendulum Binning
- Pendulum GP Modelling
- Pendulum IOC Discrete
- Pendulum IOC Continuous

### Pendulum Binning

To discretise the state space and obtain direte pmfs for state transition, we leverage the code provided in the _Probabilistic design of optimal sequential decision-making algorithms in learning and control_ ([see this link](https://arxiv.org/abs/2201.05212)). For more detailed explanation see [link](https://github.com/GIOVRUSSO/Control-Group-Code/tree/master/Decision-making). We export the discrete transition pnfs to *Discrete_Pendulum_model.zip*.

### Pendulum GP Modelling 
This file produces Gaussian process (GP) models of the system dynanics. First, we build a dataset of 3000 state-input pairs by randomly intialising the system and apllying applying inputs. Next, we choose the kernel function and initialise it's parameters. We split the dataset into training and testing batches. After training the GP model we test the model's accuracy on the test set. We export the model binaries to *Target_GP_Model.dump* and *Reference_GP_Model.dump*.

### Pendulum IOC Discrete 
This code implememts the discrete case of the running example of the manuscript. It takes Dicrete pmfs of the target and reference pendulum as input. We also define model predictive control (MPC) scheme for reference pendulum with mass =0.5kg. First, part of the code computes the policy given the actual state cost. We simulate the pendulum stablisation task by applying the inputs sampled from the policy. We conduct 20 simulations and collect the state-input trajectory data. Next, we define the feature set as given in the manuscript. Using the collected data we solve a convex optimisation problem for cost estimation. The cost is reconstructed using the obtained weights.
In the last section of the code we generate the plots of the angular position ($\theta_k$) and the input ($u_k$).


#### Pendulum IOC Continuous 

This code implememts the continuous case of the running example of the manuscript. It takes GP models of the target and reference pendulum as input. To compute the integrals over the Gaussian distribution obtained from GP modelling, we use monte carlo sampling. Specifically, we calculate the average over 500 samples from the distribution. The rest of the code structure remains the same as the discrete case.
