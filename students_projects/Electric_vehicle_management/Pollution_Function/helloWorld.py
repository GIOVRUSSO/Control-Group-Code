from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
from pulp import*

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

def run():
    """execute the TraCI control loop"""
    step = 0

    Emissioni=[]
    Velocita=[]
    Distanza=[]
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if step >=0 and step <30:
            Emissioni.append(traci.vehicle.getCO2Emission("vehicle_0"))
            Velocita.append(traci.vehicle.getSpeed("vehicle_0"))
            Distanza.append(traci.vehicle.getLanePosition("vehicle_0"))
        step+=1

    print(Emissioni)
    print(Velocita)
    print(Distanza)
    
    traci.close()
    sys.stdout.flush()
	
	
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/helloWorld.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
