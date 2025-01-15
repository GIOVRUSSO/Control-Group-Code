This folder contains the code for the experiments in the simulated Robotarium environment.
The optimal weights are computed by solving the minimization problem illustrated in the paper at each time-step $k$:
```math
\begin{split}
    \min_{\mathbf{w}_{k}} \quad & {D}_{KL} \left(p_{k}\left(\mathbf{x}_{k},\mathbf{u}_{k}\mid \mathbf{x}_{{k-1}}\right) \,\big\|\, q_{k}\left(\mathbf{x}_{k},\mathbf{u}_{k}\mid \mathbf{x}_{k-1} \right) \right) + \mathbb{E}_{p_{k}\left(\mathbf{x}_{k},\mathbf{u}_{k}\mid \mathbf{x}_{k-1}\right)}\left[c_{k}^{(x)}\left(\mathbf{X}_{k}\right) + c_{k}^{(u)}\left(\mathbf{U}_{k}\right) + l_{k}^{\star}\left(\mathbf{X}_{k-1}\right) \right]\\
    \mbox{s.t.} & \ p_{\mathbf{u},{k}}\left(\mathbf{u}_{{k}}\mid \mathbf{x}_{{k-1}} \right)  = \sum_{i\in \mathcal{P}}\mathbf{w}_{k}^{(i)} \pi^{({i})}\left(\mathbf{u}_{k}\mid \mathbf{x}_{k-1} \right), \\
    & \sum_{i\in \mathcal{P}}  \mathbf{w}_{k}^{(i)}=1,\\ 
    &  \mathbf{w}_{k}^{(i)}\ge 0, \ \forall i \in \mathcal{P}.
\end{split}
```
In order to improve the computational efficiency, the cost function is rewritten, by exploiting the definitions and substituting the first constraint $` p_{\mathbf{u},{k}}\left(\mathbf{u}_{{k}}\mid \mathbf{x}_{{k-1}} \right)  = \sum_{i\in \mathcal{P}}\mathbf{w}_{k}^{(i)} \pi^{({i})}\left(\mathbf{u}_{k}\mid \mathbf{x}_{k-1} \right) `$, as follows:
```math
\begin{split}
&{D}_{KL} \left( p_{\mathbf{x},{k}} \left(\mathbf{x}_{{k}}\mid \mathbf{x}_{{k-1}}, \mathbf{u}_{{k}} \right)  \,\big\|\,   q_{\mathbf{x},{k}} \left(\mathbf{x}_{{k}}\mid \mathbf{x}_{{k-1}}, \mathbf{u}_{{k}} \right) \right) +\\
&\sum_{i \in \mathcal{P}} \mathbf{w}_{k}^{(i)} \mathbb{E}_{\pi^{({i})}\left(\mathbf{u}_{{k}}\mid \mathbf{x}_{{k-1}} \right)} \left[
    {D}_{KL} \left(p_{{k}}\left(\mathbf{x}_{{k}}, \mathbf{u}_{{k}} \mid \mathbf{x}_{{k-1}} \right) \,\big\|\, q_{{k}}\left(\mathbf{x}_{{k}}, \mathbf{u}_{{k}} \mid \mathbf{x}_{{k-1}} \right)\right)
    + c_{k}^{(u)}\left(\mathbf{U}_{k}\right)
    + \mathbb{E}_{p_{\mathbf{x},{k}} \left(\mathbf{x}_{{k}} \mid \mathbf{x}_{{k-1}}, \mathbf{u}_{{k}} \right)} \left[ c_{k}^{(x)}\left(\mathbf{X}_{k}\right) + l_{k}^{\star}\left(\mathbf{X}_{k-1}\right) \right]
\right]
\end{split}
```
The solution is found by using [CVXPY](https://www.cvxpy.org) with [SCS](https://www.cvxgrp.org/scs/).

The code produces the following plots:
  - The heat map of the cost function
   
    ![robot_cost_map](https://github.com/user-attachments/assets/e1d1d6f3-8cc5-453c-982d-80ee2df929b3)

  - The trajectories of the robot from 5 different initial conditions
    
    ![trajectories](https://github.com/user-attachments/assets/c09ff9d0-84a6-45e9-8803-fdd341e796ef)

  - The euclidean distance from the goal and the weights for each simulation

    ![results_simulation1](https://github.com/user-attachments/assets/a4dded87-9dcc-4b4c-9e41-eaea42ce5b0e)

    
