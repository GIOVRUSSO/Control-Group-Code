These files contain the code for running Scenarios $1.a$, $1.b$ and $1.c$.

### To run the code
To run these simulation, one needs a working SUMO and TraCI installation. Then, clone this repository and extract _sumo\_files.zip_ in a folder with the same name. Then, generate agent and foe files with the appropriate simulation launcher and run the simulation file.


### Repository contents
·The zip file _sumo\_files.zip_ contains the auxiliary files for the SUMO simulation, namely the ones used to describe the campus road network. It has to be unzipped into a folder called _sumo\_files_ before use.\
·The files _agent.py_ and _crowdsourcing.py_ containt the respective classes for the agent (namely, a controlled car eauipped with CRAWLING) and the controller used in CRAWLING.\
·The file _behaviors.npy_ contains the stochastic policies of each of the primitives available to the decision-maker.
#### Scenario-specific files
·The notebooks _Simulation launcher.ipynb_ are used to write the setting files _agents.npy_ and _foes.npy_ and to launch the simulations. There is one launcher per Scenario. For Scenarios $1.a$ and $1.b$n the number of connected cars within the simulation can be set by changing the argument of the simfiles function when it is called.\
·The setting files _agent.npy_ and _foe.npy_ contain information on the number, departing time and place and target destination for controlled and uncontrolled cars, respectively. We provide as example such files for Scenario $1.a$, but in general they must be generated before simulating.\
·The files _main.py_ is the main file for the simulation. They contain all remaining code useful for the simulations (such as state space calculation and reward setting, see the paper), and interface with SUMO through TraCI. They also log results once simulations are done. There are three main files, one for each scenario.

### Results
Once the simulations are done, logs are stored in the same folder as the code file. The naming convention is _logs\_run_ followed by the index of the simulation. The files are in npy format. For Scenario $1.c$, the results are seen through the video. Hence, the simulation is started with a GUI. A link to this veido can be found in the readme file of this repository.
