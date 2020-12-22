Author of the scripts: S. Xie (University College Dublin)

This repo collects the scripts to replicate the numerical results of the work 'Scalability in nonlinear network systems affected by delays and disturbances' by S. Xie, G. Russo, R. Middleton. See https://sites.google.com/view/giovanni-russo/publications?authuser=0 for the preprint.

Namely:

(1) the files 'u.csv' and 'w2.csv' contain the parameters used for the simulations of the Hopfield neural network considered in Figure 4. In particular, 'u.csv' contains the constant inputs and 'w2.csv' contains the weights satisfying the sufficient conditions for scalability given in the work. Put these files in the same folder together with script 'Figure4.m' to enable 'csvread'. 

(2) the script 'Figure2_left.m' simulates the maximum distance of robotic hand position from desired position for each circle when increasing the network from 1 circle to 6 circles, which gives Figure 2, left panel;

(3) the script 'Figure2_right.m' simulates the maximum distance of robotic hand position from desired position across all the robots as a function of delay, which gives Figure 2, right panel;

(4) the script 'Figure3_top&middle.m' simulates the scalable/nonscalable robotic formation network with 14 circles (switch the parameters to get the scalable/nonscalable performance), which gives Figure 3, top panel and middle pannel;

(5) the script 'Figure3_bottom.m' simulates the unstable robotic formation network with 14 circles, which gives Figure 3, bottom panel;

(6) the script 'Figure4.m' simulates the hopfield neural network, which gives Figure 4;

(7) the script 'Figure5.m' simulates the scalable/nonscalable hopfield neural network, which gives Figure 5.

