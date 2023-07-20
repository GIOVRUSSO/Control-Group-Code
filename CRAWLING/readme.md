# Simulation code for the CRAWLING smart parking system

### Introduction

This repository regroups the code for the in-car service CRAWLING, introduced in _CRAWLING: a Crowdsourcing Algorithm on Wheels for Smart Parking_ by Émiland Garrabé and Prof. Giovanni Russo. Within CRAWLING, connected cars are routed using information crowdsourced from sources such as routing services, smart sensors or pedestrians.

The manuscript is called:
_É. Garrabé, G. Russo, "CRAWLING: a Crowdsourcing Algorithm on Wheels for Smart Parking", 2022_

This repository contains the code needed to reproduce the simulations showcased in the paper, focused on two scenarios: Scenario $1$ focuses on a fleet of cars seeking parking spaces on a university campus. The scenario is split into three sub scenarios investigating different simulated conditions. In Scenario $2$, a larger fleet instead navigates the city center of Salerno.\
The folder 'University' in this directory cointains the code for Scenario $1$, and the folder 'City' the files for Scenario $2$. Refer to the readme files within for a detailed file list.\
The SUMO simulator is used for the simulations, with our codebase being written in Python (including some notebooks). The TraCI module is used to interface between the simulator and our code.

### How to use this repository
To use these files, a working SUMO installation is needed. Python, TraCI and the main scientific computing libraries need to be installed. Finally, for Scenario $1.c$, a Twitter account with developper access is needed to replicate the simulation.\
Once again, detailed instructions can be found within each folder.\
To run Scenario $1$, choose how many cars are connected cars equipped with CRAWLING, between $0$ and $150$. Then, run the cells in _Simulation launcher.ipynb_, with the amount of connected cars passed as the argument of the function _simFiles_ in the last cell. The sizes of the subfleets and the starting road links, target behaviors of connected cars and direction of uncontrolled cars can be modified in the definition of the _simFiles_ function. Note that such road links are described by their ID, which can be found in the xml file describing the road network.\
Note that for Scenario $1.c$, the main file can directly be run from the command line, which will open the simulator GUI and allow users to play the simulation. The video obtained from running Scenario $1.c$ can be found at the following link:

[Video](https://drive.google.com/file/d/1paSX3P6brfhDbNO3AKC8QnaOUAr5vfnc/view?usp=sharing)


### Authors and contributors
Émiland Garrabé - author

### References
[1] É. Garrabé, G. Russo, _CRAWLING: a Crowdsourcing Algorithm on Wheels for Smart Parking_, 2022
