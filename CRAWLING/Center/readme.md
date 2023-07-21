This folder contains the files needed to run the simulations for Scenario $2$.

### To run the code
To run these simulation, one needs a working SUMO and TraCI installation. Then, first clone the repository, and extract _salerno\_net.zip_ into a folder with the same name. Then, generate the simulation files with _Simulation\_launcher.ipynb_ (see lower) and start the simulation.

### Repository contents
·The zip file _salerno\_net.zip_ contains the auxiliary files for the SUMO simulation, namely the ones used to describe the campus road network. It also containts the sources, encoded as a .npy file. It has to be unzipped into a folder called _salerno\_net_ before use.\
·The files _agent.py_ and _crowdsourcing.py_ containt the respective classes for the agent (namely, a controlled car eauipped with CRAWLING) and the controller used in CRAWLING.\
