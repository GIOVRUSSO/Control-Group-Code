
# Connected Cars Re-Routing through Crowdsourcing

This project deals with the problems of routing vehicles using a data-drivel control technique based on crowdsourcing and re-using data. The vehicles' aim is to reach configured destinations by selecting a street among a crowd of them according to a cost function. The algorithm is validated using Python language and by interfacing with the SUMO (Simulation of Urban MObility) software through the TraCI (Traffic Control Interface) library.\
This is the project of a master thesis in Information Engineering at the University of Salerno, focusing on the content and methodologies seen during the Data-Driven Control Design module.



## How and what to run

After cloning the project, download the follow Python libraries using pip

```bash
  traci
  numpy
  matplotlib
  pygame
  pyserial
  pnmeagps
  gtts
  tk
  xlrd == 1.2.0
```
Also, download and install the most recent version of SUMO.
According to the desired use of the software, different scripts must be run or different lines of code must be decommented.

WARNING: Before running any scenario, make sure to create the files for offline paths and behaviours. In order to do so, run the "simulation_runner.py" script by setting the following parameters in the script
```bash
  CREATE_BEHAVIOURS = True
  RUN_SIMULATION = False
```
Then run the script with 
```bash
  python simulation_runner.py
```
If you do not plan to use any offline-built data, make sure to comment the "import Crowdsourcing" line in the "crowdsourcing.py" file, also always make sure to put the "ONLINE" flag to True in the "simulation_runner.py" script.

### Infos about starts and destinations of the vehicles in the simulations

As of now, the scripts have been designed to load starting and ending edges for vehicles from text files inside their respective "ScenarioData" folders. If for any reason they need to be changed, modify the following files accordingly:
- "config.txt" includes data for starting and ending edges, so change edges labeled as "start" to modify spawning edges for vehicles, change edges labeled as "target" to modify target edges for vehicles (WARNING: changing the destinations will mess with offline-built behaviours, so make sure to re-build them if you want to change the destinations);
- "works.txt" includes a list of edges labeled as "work-in-progress areas" (where vehicles will slow down), modify it as you wish to include new or remove old "wip areas" (this will possibly mess with the algorithm performances, so be careful);
- "roundabouts.txt" includes a list of edges inside the road network defining which of them belong to roundabouts or are roundabout exits, changing this will mess with the driving directions given by the vocal assistant so it is highly recommended not to change anything (this file can be dynamically built by the "mapdata" module if desired).

### Run simulated scenario

In order to run a simulated scenario, make sure to put the "HIL" flag in the "simulation_runner.py" script to False, then decomment and comment the desired lines of code to run either 1 or multiple simulations of the same scenario (further instructions are in the "simulation_runner.py" script), and finally change the parameters of the simulation as you please by modifying the values inside the script (inside the "simulation_runner.py" script all parameters are highlighted and described). In order to change geographic areas and work-in-progress area conditions, change the "SCENARIO" parameter (acceptable values are "Unisa" and "Salerno") and set the "CONSIDER_WORKS" parameter of the call of the "single_sim" function according to your desire.\
Finally, open the command prompt and run

```bash
  python simulation_runner.py
```
If the "USE_PARAM_GUI" flag was set to True, at this point a GUI will ask you to set parameters for the configuration. Change them as you like and click on "Start Simulation" (or "Quit" if you do not wish to proceed).\
If no SUMO GUI appears after running the script and/or pressing "Start Simulation", make sure the third argument of the call of the "single_sim" function inside the script is set to True (it turns the SUMO GUI on and off).\
After that, enjoy.

### Run vehicle-in-the-loop simulation (linux users only)

Before dealing with any of the scripts, make sure your computer is receiving messages containing GPS information through the Bluetooth serial port. In order to do so, follow these steps:
1. Turn Bluetooth on on both your pc and your phone device, also connect them;
2. Open the "Bluetooth GPS Output" application on your phone device and press both "START" and "VISIBLE";
3. On your pc, look for Bluetooth availability by running the following command and checking if hci0 is not hard blocked
```bash
  rfkill list
```
3. If hci0 is not hard blocked, on your pc run the following command
```bash
  sdptool browse <yourPhoneBluetoothMACAddress>
```
4. Look for a service called "SPP_Slave" (or similarly), take note of its channel, then run the following command
```bash
  sudo rfcomm bind hci0 <yourPhoneBluetoothMACAddress> <SPPSlaveChannel>
```
5. Finally, run the following command
```bash
  sudo chmod 777 /dev/rfcomm0
```
If at any point the devices get disconnected or there is a need for disconnection, release the hci0 device by running
```bash
  sudo rfcomm release hci0 <yourPhoneBluetoothMACAddress>
```
Moving onto the scripts.

In order to run a vehicle-in-the-loop simulation, make sure to put the "HIL" flag in the "simulation_runner.py" script to True, then decomment and comment the desired lines of code to run either 1 or multiple simulations of the same scenario (further instructions are in the "simulation_runner.py" script), and finally change the parameters of the simulation as you please by modifying the values inside the script (inside the "simulation_runner.py" script all parameters are highlighted and described). Fundamental parameters to set are: 
```bash
  SCENARIO = 'Unisa'
  USE_NUM_AGENTS = True
  NUM_AGENTS = '1'
  numberofsim = [1]
```

Finally, open the command prompt and run

```bash
  python simulation_runner.py
```

If the "USE_PARAM_GUI" flag was set to True, at this point a GUI will ask you to set parameters for the configuration. Change them as you like and click on "Start Simulation" (or "Quit" if you do not wish to proceed).\
If no SUMO GUI appears after running the script and/or pressing "Start Simulation", make sure the third argument of the call of the "single_sim" function inside the script is set to True (it turns the SUMO GUI on and off).\
After that, enjoy.

### Generate plots of simulation data

In order to generate plots of data coming from simulations, go to the "plots.py" script and change folders according to where your simulation-generated data is and where you want your images to be saved (make sure to set the "SAVE_FIG" flag to True to save images), finally run  
```bash
  python plots.py
```

### Generate plots of data related to time measurements (useful for performance evaluation of the algorithm)

In order to generate plots of data related to time measurements, make sure the "time_plots.py" is in the same folder as the "time_measures.txt" and the file is not empty, finally run  
```bash
  python time_plots.py
```

### Run algorithm animation for debugging purposes

In order to visualize the paths created as behaviours for the crowdsourcing algorithm, change the values of the "start" and "end" variables to fit the debugging purpose and set the desired scenario, finally run
```bash
  python algorithm_animation.py
```
