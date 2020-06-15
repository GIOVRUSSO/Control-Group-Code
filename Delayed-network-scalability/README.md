Author of the scripts: S. Xie (University College Dublin)

This repo collects the scripts to replicate the numerical results of the work 'Scalability in nonlinear network systems affected by delays and disturbances' by S. Xie, G. Russo, R. Middleton. See https://sites.google.com/view/giovanni-russo/publications?authuser=0 for the preprint.

Namely:

(1) the files 'u.csv' and 'w2.csv' contain the parameters used for the simulations of the Hopfield neural network considered in the above work. In particular, 'u.csv' contains the constant inputs and 'w2.csv' contains the weights satisfying the sufficient conditions for scalability given in the work. Put these files in the same folder together with script 'Neural_network_scalability.m' to enable 'csvread'.  

(2) the script 'Neural_network_scalability.m' simulates the hopfield neural network;  

(3) the script 'Robotic_formation_scalability.m' simulates the robotic formation network considered in one of the examples;

(4) the script 'upper_boundvs_increasingdelay.m' plots the upper bound of the state deviation for the robots when the delay increases.  

PS. the disturbance in each simulation is generated with random amplitudes, which might leads to a slight difference in the amplitude of deviation to that shown in the work.  
