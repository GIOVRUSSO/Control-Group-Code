# Stabilizing a pendulum to unstable equilibrium
This folder contains all the necessary code required to replicate the results of the running pendulum example of the manuscript.

### Contents 
The following files are contained in this folder,
- Pendulum Binning
- Pendulum GP Modelling
- Pendulum IOC Discrete
- Pendulum IOC Continuous

### Pendulum Binning

To discretise the state space and obtain direte pmfs for state transition, we leverage the code provided in the _Probabilistic design of optimal sequential decision-making algorithms in learning and control_ ([see this link](https://arxiv.org/abs/2201.05212)). For more detailed explanation see [link](https://github.com/GIOVRUSSO/Control-Group-Code/tree/master/Decision-making)

### Pendulum GP Modelling 
This file produces Gaussian process (GP) models of the system dynanics. First, we build a dataset of 3000 state-input pairs by randomly intialising the system and apllying applying inputs. Next, we choose the kernel function and initialise it's parameters. We split the dataset into training and testing batches. After training the GP model we test the model's accuracy on the test set. We export the model binaries to *Target GP Model.dump* and *Reference GP Model.dumo*.
