## On the design of integral multiplex control protocols in nonlinear network systems with delays
### Introduction
This repository collects the supporting code for the manuscript **On the design of integral multiplex control protocols in nonlinear network systems with delays** (arXiv link). In the manuscript, we designed a distributed integral control protocol for a network system in the form of multiplex architecture to *reject polynomial disturbances* and proposed sufficient conditions to guarantee *scalability* of the network system with regard to the residual disturbances.

### Contents
The following list of files can be found in the repository, which reproduce the parameters/figures in the manuscript.
- **multiplex_optimization.m**

This code solves a convex optimization problem to obtain the gains for the multiplex control protocol (see Appendix B in the manuscript for details of the optimization problem). The optimal solution is visualizable in the figure (marked with 'x') produced by the code which collects all the feasible solutions to the optimization problem. At the same time, the gains ($k_0$, $k_1$, $k_2$, $g_0$, $g_1$, $g_2$) will be displayed in the command window. Knowing the value of $g_0$, $g_1$, $g_2$, we can select the control gains for the delayed couplings according to $g_0=k_0^{(\tau)}k^\psi$, $g_1=k_1^{(\tau)}k^\psi$, $g_2=k_2^{(\tau)}k^\psi$, e.g. let $k^\psi=0.1$, then $k_0^{(\tau)}=10g_0$, $k_1^{(\tau)}=10g_1$, $k_2^{(\tau)}=10g_2$.

- **max_hand_position_deviation.m**


- **in_silico_validation.m**
- **robotarium**
  - **robotarium_validation.m**
  - **posdev_hardware.mat**
  - **stepsize_hardware.mat**
  - **all other utilities to get the code run**

### A dynamic view of hand position deviation via Flourish
https://public.flourish.studio/story/1572969/

### Author and contributer
Shihao Xie (shihao.xie1@ucdconnect.ie)

### Reference
*Shihao Xie, Giovanni Russo. On the design of integral multiplex control protocols in nonlinear network suystems with delays, 2022.*
