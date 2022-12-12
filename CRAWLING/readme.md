# Simulation code for the CRAWLING smart parking system

### Introduction

This repository regroups the code for the in-car service CRAWLING, introduced in _CRAWLING: a Crowdsourcing Algorithm on Wheels for Smart Parking_ by Émiland Garrabé and Giovanni Russo. Within CRAWLING, connected cars are routed using information crowdsourced from sources such as routing services, smart sensors or pedestrians.

The draft is called:
_É. Garrabé, G. Russo, "CRAWLING: a Crowdsourcing Algorithm on Wheels for Smart Parking", 2022_

This repository contains the code needed to reproduce the simulations showcased in the paper, focused on two scenarios: in Scenario $1$, a fleet of students arrives on the University of Salerno at rush hour and seeks to find parking spaces on campus. In Scenario $2$ two cars are considered, both trying to achieve the same goal. However, a road obstruction makes this impossible for the second car. When pedestrians notice the obstruction, they share it on social media and the in-car service adapts to the event.\
The SUMO simulator is used for the simulations, with our codebase being written in Python (including a notebook). The TraCI module is used to interface between the simulator and our code.

### Repository contents
·The zip file _sumo\_files.zip_ contains the auxiliary files for the SUMO simulation, namely the ones used to describe the campus road network. It has to be unzipped into a folder called _sumo\_files_ before use.\
·The files _agent.py_ and _crowdsourcing.py_ containt the respective classes for the agent (namely, a controlled car eauipped with CRAWLING) and the controller used in CRAWLING.\
·The file _behaviors.npy_ contains the stochastic policies of each of the primitives available to the decision-maker.
#### Scenario $1$-specific files
·The notebook _Simulation launcher.ipynb_ is used to write the setting files _agents.npy_ and _foes.npy_ and to launch the simulations.
·The setting files _agent.npy_ and _foe.npy_ contain information on the number, departing time and place and target destination for controlled and uncontrolled cars, respectively.\
·The file _main.py_ is the main file for the simulation. It contains all remaining code useful for the simulations (such as state space calculation and reward setting, see the paper), and interfaces with SUMO through TraCI. It also logs results once simulations are done.
#### Scenario $2$-specific files
·_main_twittersim.py_ is used to handle the simulation of Scenario $2$. As there are only two connected cars, they are directly handled in this file. The file also contains the tweet parser and interface to Twitter. A last difference is that this file calls up a graphical interface.\
·_simvid.webm_ is the simulation video for Scenario $2$. It showcases the first connected car passing through the highway ramp, which is then obstructed (and colored in blue). The second connected car then takes an alternate route to avoid the obstruction.

### How to use this repository
To use these files, a working SUMO installation is needed. Python, TraCI and the main scientific computing libraries need to be installed. Finally, for Scenario $2$, a Twitter account with developper access is needed to replicate the simulation.\
To run Scenario $1$, choose how many cars are connected cars equipped with CRAWLING, between $0$ and $150$. Then, run the cells in _Simulation launcher.ipynb_, with the amount of connected cars passed as the argument of the function _simFiles_ in the last cell. The sizes of the subfleets and the starting road links, target behaviors of connected cars and direction of uncontrolled cars can be modified in the definition of the _simFiles_ function. Note that such road links are described by their ID, which can be found in the xml file describing the road network.\
For Scenario $2$, the nain file can directly be run from the command line, which will open the simulator GUI and allow uers to play the simulation. Scenario $2$ yields the following behavior, which can also be found in this repository as _video\_scenario2.webm_.

[video_scenario2.webm](https://user-images.githubusercontent.com/10179753/207033728-0892432a-62cd-4403-8b82-1e43614a6dbe.webm)



### Authors and contributors
Émiland Garrabé - author

### References
[1] É. Garrabé, G. Russo, _CRAWLING: a Crowdsourcing Algorithm on Wheels for Smart Parking_, 2022
