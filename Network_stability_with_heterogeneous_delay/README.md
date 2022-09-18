## On the design of multiplex control to reject disturbances in nonlinear network systems affected by heterogeneous delays
### Introduction
This repository collects the supporting code for the manuscript **On the design of multiplex control to reject disturbances in nonlinear network systems affected by heterogeneous delays**. In the manuscript, We consider the problem of designing control protocols for nonlinear network systems affected by heterogeneous, time-varying delays and disturbances. We illustrate the effectiveness of the results via a numerical example that involves the control of a multi-terminal high-voltage DC (MTDC) grid. Here we report all the code and information to replicate all our results in the manuscript. The code can also be used to apply our methodology to different settings. If you use the code, please reference our work.

### Contents
The following list of files can be found in the repository, which reproduce the parameters/figures in the manuscript.
- **optimisation_problem_for_parameters.m**

This code solves a convex optimization problem to obtain the gains for the multiplex integral control protocol. Directly running the code gives the control gains ($k_0$, $k_1$, $k_2$, $g_0$, $g_1$, $g_2$) for the control of MTDC gird.


- **MTDC_control.m**

This code simulates the problem of controlling a grid of 5 terminals. All the communications between terminals are affected by randomly generated heterogeneous delays and one terminal is affected by a disturbance with a first order component. The control goal is to have the voltage deviation of all terminals reaching $0$.

- **Multiplex architecture**

The figure illustrates the architecture of the multiplex integral control protocol we use.

- **MTDC structure**

The figure illustrates the ring topology of the MTDC grid with $5$ terminals.

### Results

- **voltage deviation and control signal**

The top panel of the figure shows the voltage deviation of all the terminals. All the deviation converge to $0$ as expected including the pertured terminal $1$. The bottom panel shows the control input for each terminal, including a ramp control signal which compensates for the ramp disturbance to terminal $1$.

### Author and contributer
Shihao Xie (shihao.xie1@ucdconnect.ie)

### Reference
*Shihao Xie, Giovanni Russo. On the design of multiplex control to reject disturbances in nonlinear network systems affected by heterogeneous delays, 2022.*





