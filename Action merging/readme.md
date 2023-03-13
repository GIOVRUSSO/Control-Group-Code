# On Action Composition for Autonomous Sequential Decision-Making

### Introduction
An attractive idea in stochastic control is that of obtaining policies from a pool of experts or sources. In the paper _On Action Composition for Autonomous Sequential Decision-Making_ this idea is expounded upon by allowing an agent to merge such sources to obtain its behaviors (arXiv link pending). This repository regroups the code used to perform the simulation experiments supporting the paper.

_É. Garrabé, M. Lamberti, G. Russo, On Action Composition for Autonomous Sequential Decision-Making, 2023_

### Experiment description
At rush hour, a fleet of connected cars arrive on the campus of the University of Salerno, with drivers seeking a parking space.

### Repository contents
In this repository, we provide all the files needed to replicate the numerical experiments in the paper, and the video showcasing our Hardware-in-the-Loop experiment. Namely, the files are:
- _sumo\_files.zip_ contains all the files describing the campus road network in SUMO.
- _main.py_ is the main simulation file for the cars controlled by a Crowdsourcing decision-maker.
- _main\_merge.py_ is the main simulation file for cars Controlled by the decision-maker described in the paper.
- _crowdsourcing.py_ features the class where both decision-makers are implemented.
- _behaviors.npy_ contains the sources, repository of all the stochastic behaviors to which the connected cars have access. Such behaviors are encoded as discrete probability functions.
- _HIL\_route.mp4_ is the video obtained after running our HIL experiments. It features, side-by-side, a capture of the SUMO simulator, of a smartphone displaying the car's position in a GPS application, and live footage from the passenger's perspective.

### Implementation details
