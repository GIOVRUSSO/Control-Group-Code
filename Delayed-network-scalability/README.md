Author of the scripts: S. Xie (University College Dublin)

This repo collects the scripts to replicate the numerical results of the work 'Scalability in nonlinear network systems affected by delays and disturbances' by S. Xie, G. Russo, R. Middleton. See https://sites.google.com/view/giovanni-russo/publications?authuser=0 for the preprint.

Namely:

(1) the files 'u.csv' and 'w2.csv' contain the parameters used for the simulations of the Hopfield neural network considered in Figure 4. In particular, 'u.csv' contains the constant inputs and 'w2.csv' contains the weights satisfying the sufficient conditions for scalability given in the work. Put these files in the same folder together with script 'Neural_network_Fig4.m' to enable 'csvread'. 

(2) the script 'deviations_delay_Fig1.m' simulates the maximum deviation of the robotic hand position across all the robots as a function of delay, which gives Figure 1, right panel;

(3) the script 'Robotic_formation_scalability_Fig2.m' simulates the robotic formation network with 6 circles, which gives Figure 2.

(4) the script 'Robotic_formation_scalability_Fig3_neighbourconnection.m' simulates the robotic formation network with 14 circles, which gives Figure 3, top panel and middle pannel;

(5) the script 'Robotic_formation_scalability_Fig3_fullyconnected.m' simulates the robotic formation network with 14 circles, which gives Figure 3, bottom panel;

(6) the script 'Neural_network_Fig4.m' simulates the hopfield neural network, which gives Figure 4;

(7) the script 'Neural_network_Fig5.m' simulates the scalable/nonscalable hopfield neural network, which gives Figure 5.



PS. the disturbance in part of the simulations is generated with random amplitudes, which might leads to a slight difference in the amplitude of deviation to that shown in the work.  
