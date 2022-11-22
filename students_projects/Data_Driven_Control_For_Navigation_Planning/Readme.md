# Data Driven Control for navigation planning

This repository collects all the work done during the thesis activities for the Master's Degree in Computer Engineering. The thesis concerns Data Driven Control Systems Design, in particular Data Driven Control for Navigation planning. Nowadays, transportation systems are a key part of human activities and engineering infrastructure is needed to cope with the increase in vehicles on the roads. It becomes necessary to promote safer roads by limitating the risk of accidents and reducing delays time, together with fuel consumption and emission rates. At the same time, a smart parking system is needed to ensure the driver a good driving experience, avoiding traffic jams and wasting time. The **goal of this thesis is the engineering of a data driven control algorithms** that leverage on use of data coming from the surrounding environment. The **validation** took place in two urban scenarios: Fisciano Campus and Salerno centre. The simulations considered a typical morning when students, professors and employees need to park in one of three main parking lots in Fisciano Campus. Salerno centre is a more complex scenario that takes into account twenty-one parking spaces, distributed among the most known areas: during Christmas Lights, traffic jams are very frequent and the streets are very crowded. For the University of Salerno scenario, the **Hardware-in-the-Loop approach** was followed, together with **SUMO simulations**. The various scenarios are organized into folders.
The repository contains three main folders:

* **SalernoScenario** contains the urban scenario of Salerno centre validated in SUMO.
* **UnisaScenario** contains the digital twin of the University of Salerno validated in SUMO.
* **UnisaHIL** regards the Hardware-in-the-Loop validation that took place in Fisciano Campus (University of Salerno).

Each folder is organized as follows:

* **subfolder called unisa_net or salerno_net** contains all the files useful for the generation of road networks and traffic demand.
* **agent.py** contains the class definition for an Agent object (the term agent refers to the controlled car by control algorithm).
* **crowdsourcing.py** contains the class definition for a Crowdsourcing object, which is characterized by the decision-making function implementation.
* **demo_crowds.py** contains the main script to test the crowdsourcing algorithm on the corresponding scenario.
*  * The user configure the total number of cars and capacity of parking spaces.
* **utility.py** contains utility functions such as reward functions, rerouting functions etc.
* **create_behaviors.ipynb** is the Jupyter Notebook that creates the target behaviors for the crowdsourcing algorithm.
  * The user configure which are the target edges that controlled cars have to reach. In the context of this thesis, targets are parking areas.
* **Simulationlauncher.ipynb** is the Jupyter Notebook that creates the controlled and uncontrolled cars sets and run simulations in SUMO. 
  * The user configures the total number of cars, together with the number of controlled cars by the crowdsourcing algorithm. 
  * In addition, starting edges from which cars are shuffled spawned into the simulations are defined. 
  * For the controlled cars, it is important to set up the goal, that corresponds to a target parking area.
  * Uncontrolled cars have a pre-established route, given the starting and ending edge.
* **plots.ipynb** is the Jupyter Notebook that allow to obtain diagrams and evaluation metrics to validate the crowdsourcing algorithm.

## How to use:

1. Configure and run **create_behaviors.ipynb**
2. Configure **demo_crowds.py**
3. Configure and run **Simulationlauncher.ipynb**
4. Configure and run **plots.ipynb**
 
## Author:

Martina Lamberti (Master Student @ University of Salerno)
