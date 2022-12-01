###Introduction

In the paper _CRAWLING: a Crowdsourcing Algorithm on Wheels for Smart Parking_ by Émiland Garrabé and Prof. Giovanni Russo, we propose the principled design of an in-car routing service, where connected cars are routed using information crowdsourced from sources such as routing services, smart sensors or pedestrians.\

_É. Garrabé, G. Russo, "CRAWLING: a Crowdsourcing Algorithm on Wheels for Smart Parking", 2022_\

This repository contains the code needed to reproduce the simulations showcased in the paper, focused on two scenarios: in Scenario $1$, a fleet of students arrives on the University of Salerno at rush hour and seeks to find parking spaces on campus. In Scenario $2$ two cars are considered, both trying to achieve the same goal. However, a road obstruction makes this impossible for the second car. When pedestrians notice the obstruction, they share it on social media and the in-car service adapts to the event.\

###Repository contents
·The subrepository _sumo\_files_ contains the auxiliary files for the SUMO simulation, namely the ones used to describe the campus road network.\
·The files _agent.py_ and _crowdsourcing.py_ containt the respective classes for the agent (namely, a controlled car eauipped with CRAWLING) and the controller used in CRAWLING.\
·The file _behaviors.npy_ contains the stochastic policies of each of the primitives available to the decision-maker.\
####Scenario $1$-specific files
·The notebook _Simulation launcher.ipynb_ is used to write the setting files _agents.npy_ and _foes.npy_ and to launch the simulations.\
·The setting files _agent.npy_ and _foe.npy_ contain information on the number, departing time and place and target destination for controlled and uncontrolled cars, respectively.\
·The file _main.py_ is the main file for the simulation. It contains all remaining code useful for the simulations (such as state space calculation and reward setting, see the paper), and interfaces with SUMO through TraCI. It also logs results once simulations are done.
####Scenario $2$-specific files
·_main_twittersim.py_ is used to handle the simulation of Scenario $2$. As there are only two connected cars, they are directly handled in this file. The file also contains the tweet parser and interface to Twitter. A last difference is that this file calls up a graphical interface for result isualisation.
·_simvid.webm_ is the simulation video for Scenario $2$. It showcases the first connected car passing through the highway ramp, which is then obstructed (and colored in blue). The second connected car then takes an alternate route to avoid the obstruction.

###How to use
To use these files, a working SUMO installation is needed. Python, TraCI and the main scientific computing libraries need to be installed. Finally, for Scenario $2$, a Twitter account with developper access is needed to replicate the simulation.
