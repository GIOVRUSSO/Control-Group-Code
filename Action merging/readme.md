# On Action Composition for Autonomous Sequential Decision-Making

### Introduction
An attractive idea in stochastic control is that of obtaining policies from a pool of experts or sources. In the paper _On Action Composition for Autonomous Sequential Decision-Making_ this idea is expounded upon by allowing an agent to merge such sources to obtain its behaviors (arXiv link pending). This repository regroups the code used to perform the simulation experiments supporting the paper.
To replicate our experiments, a working installation of SUMO, Python and the appropriate libraries (TraCI, usual libraries: Numpy, Matplotlib...) is required.

_É. Garrabé, M. Lamberti, G. Russo, On Action Composition for Autonomous Sequential Decision-Making, 2023_

### Experiment description
At rush hour, a fleet of connected cars arrive on the campus of the University of Salerno, with drivers seeking a parking space. $100$ cars enter the campus at $5$-second intervals. The cars follow directions output by the decision-making algorithm introduced in the paper, while another fleet follows a crowdsourcing decision-maker as a benchmark.

### Repository contents
In this repository, we provide all the files needed to replicate the numerical experiments in the paper, and the video showcasing our Hardware-in-the-Loop experiment. Namely, this repository contains:
- _sumo\_files.zip_ contains all the files describing the campus road network in SUMO. It should be extracted into a folder called _sumo\_files_ before running simulations. 
- _main.py_ is the main simulation file for the cars controlled by a Crowdsourcing decision-maker. The code within reads information files on the simulation, interfaces with the simulator through TraCI and, at the end of the simulation, logs the results.
- _main\_merge.py_ is the main simulation file for cars Controlled by the decision-maker described in the paper.
- _crowdsourcing.py_ features the class where both decision-makers are implemented.
- _agent.py_ features the class dedicated to representing a connected car within the simulation.
- _agent.npy_ is a numpy array containing all the information about agents in the simulation. Such information is read by the main Python files at the beginning of the simulation and includes initial road link, departing time and target behavior of the agents.
- _foe.npy_ contains similar information for uncontrolled cars. Such cars follow SUMO's automated routing, which is precomputed when they enter the simulation. While not useful in this case as simulations only include controlled cars, reading and writing to this file is still supported to allow users to customize simulations.
- _behaviors.npy_ contains the sources, repository of all the stochastic behaviors to which the connected cars have access. Such behaviors are encoded as discrete probability functions.
- The notebook _Simulation\_launcher.ipynb_ allows user to write on the files _agent.npy_ and _foe.npy_ by specifying the number of both types of cars to be included in a simulation, as well as their starting road link, departure times and individual goals. One can also directly program batches of simulations from this file for convenience.
-- The video embedded in the next section, _HIL\_route.mp4_ was obtained after running our HIL experiments. It features, side-by-side, a capture of the SUMO simulator, of a smartphone displaying the car's position in a GPS application, and live footage from the passenger's perspective. If the embedding lower doesn't work, the video can also be reached through the following link: https://drive.google.com/file/d/1XPu_6-Fd-ZyGrnSD6JCfJ-7YQYguKcW8/view?usp=sharing

### Video
[[Watch the video (external link)]![video thumbnail](https://user-images.githubusercontent.com/10179753/225648231-535825b4-60af-4252-88eb-01eaeacde6ec.png)
](https://drive.google.com/file/d/1XPu_6-Fd-ZyGrnSD6JCfJ-7YQYguKcW8/view?usp=sharing)

### Implementation details
This section gives further details on the implementation of the simulations and HIL experiment.

##### Source calculation
The sources are a repository of behaviors, from which connected cars can compose a behavior. In the paper, the sources are probability mass functions (pmfs), containing the conditional probabilities of a car merging into adjacent road links.
Each source contains directions to a different section of the campus. This is done to ensure they provide varied choices to the agent on each link. In this work we choose three sources, two of them leading the agent to each of the parking lots, and the last towards the southern section of the campus. For each such direction and for each link, the corresponding source is obtained by computing the next road link on the route towards the destination (using SUMO's routing) and assigning a high probability to this road link, before adding uniform noise to all the other possible turns. This noise represents driver error or a requirement for user privacy, and fulfils Assumption 1 of the paper (when one considers as state space the set of adjacent links).

##### State space restriction
We only compute the weights of Algorithm 1 for the road links that can be reached from the agent's position in $N$ time steps or less, where $N$ is the algorithm's time horizon. Indeed, links that are farther away will not have any impact on the current decision. This is due to the way the sources are computed, with only adjacent links being assigned non-zero probability. This trick significantly reduces the computation load of the decision-maker, with a decision being achieved in about half a second, which is a realistic time frame for this application.

##### HIL implementation
A standard smartphone is used as GPS sensor for this experiment. One could also buy a dedicated sensor or the one included in some modern laptop, but this approach is more general. The GPS data is sent to a laptop using an application, and stored in an rfcomm file. This file is read using the pySerial library, and translated in NMEA format. This is finally used to place a virtual avatar of the car in the simulation. When a road link change is detected, the controller is queried, and the output is sampled, with the given road link being highlighted in the simulation GUI. The driver then follows such directions. This functional pipeline is illustrated in the following figure:
![image](https://user-images.githubusercontent.com/10179753/225348461-59f6bd60-9d7f-44a4-bcc6-27c7b0fa022a.png)

Finally, as explained in the file summary, the simulation video contains a side-by-side depiction of the simulation GUI, screen capture of a GPS application displaying the car's position, and passenger footage.

Repository code author: Émiland Garrabé. HIL experiments development and execution: Martina Lamberti.
Contact: egarrabe@unisa.it
