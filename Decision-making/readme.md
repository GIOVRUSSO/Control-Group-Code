# Tutorial in Probabilistic Design for Learning and Control
### Introduction
Probabilistic Design aims at developping optimal control policies from data. This repository concentrates the supporting code for the review and tutorial paper _Probabilistic design of optimal sequential decision-making algorithms in learning and control_ (https://arxiv.org/abs/2201.05212).

_É. Garrabé, G. Russo, "Probabilistic design of optimal sequential decision-making algorithms in learning and control", 2022_

### Repository contents
The following files can be found in the repository:
**·Pendulum binning**
**·Pendulum MPC**
**·Pendulum FPD**
**·Pendulum QLearning**
**·Pendulum KLC**
**·Histogram binning** (generic code, see Annex for tutorial)

##### Pendulum binning
While the general code used to perform histogram binning is provided on this github, we propose here to see it in action as a tool to estimate the plant of two different pendulums. The code also serves as a general example for offline estimation, with the data being generated before being binned.
The pendulum class is simply used to generate the simulation data, with all the parameters (from mass to noise deviation and time discretization) being modifyable.
The _database_ is then generated through 10.000 100-step episodes. The state is randomly initialized at the beginning of each episode, and the inputs are random, which has the advantage of providing a uniform-coverage database that is descriptive of the actual dynamics. Long episodes or non-unfiform policies would _bias_ the data generation process. This amount of data points makes the binning process somewhat lengthy (an order of an hour) but is the empirical minimum to get descriptive pfs given the state space's dimension and discretization.
The data is then binned into arrays that represent marginal probabilities which are finally used to estimate the conditionnal plant pmfs using Bayes' rule.
Finally, the plant pmfs corresponding to two pendulums (m=0.5kg and m=1kg) are stored as .npy files.
For simplicity's sake, we also provide the binned plants as .npy files in the plants.zip files. Keep in mind that the files need to be extracted in the same folder as the jupyter notebooks and that they are quite heavy (about a gigabytes each).

##### Pendulum MPC
Using the same pendulum class as before, an MPC controller is used on the pendulum. The library do-mpc is used for this controller, which has a cost of $\theta_k^2 + 0.1\gamma_k^2$ and a termminal cost of $\theta_k^2 + 0.5\gamma_k^2$, where $\theta_k$ and $\gamma_k$ are respectively the angular positin and speed at time $k$. The time horizon is 20.
The actions are then supplemented with gaussian noise (std = 0.2) and discretized, which is in practice equivalent to sampling them from a stochastic policy obtained from the action returned by the MPC.
Fifty simulations are then performed to validate the results, which are used as data for the (mean, std) angular position plot shown in the paper.

##### Pendulum FPD
The pendulum used for the MPC simulations has a mass of 0.5kg. This file is centered around using the _Control from demonstrations_ subset of FPD to adapt the corresponding MPC policy to the control of a 1kg pendulum. This is done by adding an _MPC-step_ method to the Pendulum class, which, from an MPC action, derives the expert policy before applying FPD to derive the actual policy and applying it to the pendulum, performing a simulation step in the process.
Similarly to MPC, 50 simulations are performed as validation.

##### Pendulum QLearning
The well-known tabular Q-Learning algorithm is applied to the pendulum. In line with the benchmarking aspect of the experiment, the discretization and cost signal are identical to the ones used for FPD.
The algorithm uses a discounted approach, with a discount factor of 0.99 and a learning rate of 0.5. Learning is performed using 500-step episodes, and checkpoints are regularly used to evaluate the performance up until the agent raches a total of 100.000 learning episodes. The learning policy is $\epsilon$-greedy, with $\epsilon = 0.9$. At each checkpoint, the performance is evaluated using 300-episode simulations using the greedy policy. This is used to plot the (mean-std) reward at each checkpoint and to plot graphs similar to the ones done with MPC and FPD.

##### Pendulum KLC
The final algorithm used is the KL-Control. First, the system's passive dynamics are obtained by extracting the pmf $f^{(x)}(x_k|0,x_{k-1})$ for all $x_{k-1}$.
Then, the states are enumerated from 1 to 2500, and this enumeration is used to build the matrix diag(exp(-q)) (where q is the same cost as used for the MPC and Q-Learning) and the transition probability matrix P, which is obtained from the previously-described passive dynamics.
Finally, the eigenvector problem is solved using the power method. Here, the amount of iterations (50) is determined empirically for this specific problem, but in general we recomment using the following stopping condition: in two successive iterations, the norm of the resulting vectors is equal up to some tolerance.
With the newly calculated z-function, one can then derive the optimal transitions and perform the usual 50 simulations.

### Concluding remarks
While in general we do not propose here the general form of the studied control algorithms, this repository can be seen as a showcase of their implementation, from system description and implementation to performance analysis.
An overarching function that can be found in many of the files is the discretize function. It simply finds to which bin a value (or vector) belongs according to the discretization parameters that are given as inputs.

### Authors and contributors
**·Émiland Garrabé** (egarrabe@unisa.it) - author.

### References
[1] É. Garrabé, G. Russo, "Probabilistic design of optimal sequential decision-making algorithms in learning and control", 2022 ([arXiv](https://arxiv.org/abs/2201.05212))

### Annex: Histogram binning guide
While technically not used for the project, we include the general code for the histogram binning in this repository. The following is the initially-written guide to use this code. a description of the algorithm and general good practices can also be found in the paper.

_________________________________________________________________

The code implements the histogram binning method for estimating the general probability mass function f(z|y). A utility is also provided to convert system data into a format useable with the previous code. Finally, the principle is illustrated through a numerical example. This file is organised in three sections that reflect this structure.

I/ Binning code
Code for the binning of the generic probability mass function f(z_k|y_{k-1}).
The binning is done through two functions, getJointPMFs and getConditionnal. There is also the auxiliary function discretize which returns the discrete index for a multi-dimensional variable.
Aside from the data, which will be explained further later, the program needs the following information:
- dimension of the variables. This is an integer (in the code, Zdim and Ydim).
- minimum values of the variables along each axis. List containing the minimal values for each dimension of the variable (in the code, Zmin and Ymin).
- discretization step along each axis. List of the discretization steps for each dimension (in the code, Zstep and Ystep).
- amount of discrete bins along each axis. List of the amount of bins in the discretization of the variable along each dimension (in the code, Zdiscr and Ydiscr).
See the code for an example (detailed lower) of these variables being declared and used. Note that we do not explicitly provide maximum values to the algorithm. This means that ensuring that the data fits into the correct bounds is up to the user.

Following the idea of histogram binning, two joint pmfs are created. The first one, f(z,y), tracks joint occurences of z and y in the data set, while f(y) simply tracks y. By iterating over the trajectories in the dataset, the algorithm estimates these two pmfs.
The dataset should take the following form: D = [[Z1,Y1],[Z2,Y2]...,[Zm,Ym]] where [Zi,Yi] is a trajectory, with Zi being of the following form: Zi = [zi1, zi2, ..., zin] and m is the amount of trajectories to be binned.
zij is the j-th element of the i-th trajectory for Z. It should be coded as a list or a tuple, even when its dimension is 1. This is important as the binning algorithm iterates over the dimensions of the variables to find the corresponding indexes. In practice, an example for a 1D Zi would be: [[1.1],[4.0],[3.9],...[2.7]].
Yi is organized similarly, with the algorithm assuming that z_k and y_{k-1} are coded at the same level in the list: Yi = [yi0, yi1, ..., yi(n-1)].
After the binning, the pmfs are normalized (in practice this isn't needed but is mathematically sound). Finally, the conditional pmf is calculated through an element-wise division. If the joint pmfs are equal to zero, the conditional is also taken to be zero.

Once computed, to get the pmf f(z_k|y_{k-1}), one simply needs to call cond[inds], where cond is the computed pmf and inds is the indexes corresponding to y_{k-1} (one can use the function discretize to go from an actual value to an index).
In the case where the pmf is the plant f(x_k|x_{k-1}, u_k), inds becomes (indsX,indsU), the concatenation of the indexes for x_{k-1} and u_k.

II/ System utility
We will now describe the function formatHistory.
In practice, we often want to use this code to bin the plant for a system. This makes Z the state, and Y the joint variable of the state and input. We assume that x_k is conditioned on u_k and x_{k-1}. We also assume that the data has the same structure as before, where trajectories are lists, each comprising a list of states and a list of inputs.
As before, the input must be a list of trajectories, themselves defined as a list containing the history for the state and for the action.
The trajectories have to be of equal length:
Xi = [xi0, xi1, ..., xin]
Ui = [ui0, ui1, ..., uin]
Note that this means that the first input of each trajectory will be discarded.
The utility reshapes the histories to fit the Z = X, Y = X,U idea. This simply means filling new trajectories with the correctly indexed states and inputs. There is an additionnal layer of safety, which converts int or float values to lists to make them compatible with the algorithm of Section I even if the state and input histories were not.

III/ Numerical example
We consider the following system: x_k = x_{k-1} + u_k + N(0,1) where N denotes the normal law.
The state and input are one-dimensional. Note how, in the code, the parameters for Y represent a 2D variable as Y is the joint variable (x_{k-1}, u_k) in the case of plant estimation.
We generate a 100000-step trajectory by randomly selecting a binary input and sampling the new state. If the state goes outside of the bound we crop it back to the considered interval (in practice this should be ensured for any data used with the algorithm).
We then store this [x,u] (again, with x and u being lists) trajectory into the data and format the data.
Note that the states and inputs are simply integers (ie u is of the form [0, 1, 1, ..., 0] instead of [[0], [1], [1], ..., [0]]. This is bad practice but highlights the safety incorporated in the reshaping algorithm.
Finally, we get the joint and conditional pmfs using the binning method and plot the result as an example.

Note that, if a user desires to use the code we just described without the example, there is also the option of copy-pasting the relevant functions into a separate python file and importing it into another python file or jupyter notebook for ease of use within a wider project.
