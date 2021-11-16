This file contains all relevant information about the code provided for the Histogram binning code.
Code and present file author: Émiland Garrabé (egarrabe@unisa.it).

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
