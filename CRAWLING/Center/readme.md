This folder contains the files needed to run the simulations for Scenario $2$.

### To run the code
To run these simulation, one needs a working SUMO and TraCI installation. Then, first clone the repository, and extract _salerno\_net.zip_ into a folder with the same name. Then, generate the simulation files with _Simulation\_launcher.ipynb_ (see lower) and start the simulation.

### Repository contents
·The zip file _salerno\_net.zip_ contains the auxiliary files for the SUMO simulation, namely the ones used to describe the campus road network. It also containts the sources, encoded as a .npy file. It has to be unzipped into a folder called _salerno\_net_ before use.\
·The files _agent.py_ and _crowdsourcing.py_ containt the respective classes for the agent (namely, a controlled car eauipped with CRAWLING) and the controller used in CRAWLING.\
.The files _agent.npy_ and _foe.npy_ contain information about cars with and without (respectively) CRAWLING. This information includes starting position, time and the goal of the car (for controlled cars this is the index of the target behavior, while for uncontrolled cars the target road link is given directly. Two example files are provided, but in general they should be regenerated before each simulation.
.The notebook _Simulation\_launcher.ipynb_ is used to create these simulation files. The first cells handle importing important libraries and defining useful functions. Then, when prompted, select how many (out of the total of $300$) cars will be equipped with CRAWLING in the simulations. The final cells create simulation files and run simulations.

### Results
Once the simulations are finished running, the logs are saved in ./salerno\_net/logs. The log filezs are called _salerno\_unparked_ followed by the id of the simulation and are in .npy format.
