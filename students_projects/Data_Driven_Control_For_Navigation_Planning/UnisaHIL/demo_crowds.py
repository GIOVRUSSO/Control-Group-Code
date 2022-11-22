from datetime import datetime
from threading import Thread
from time import sleep
from utility import change_route_color,computeReward, getEdgeIndex, getFreeParking, get_options
from utility import add_agents
from utility import add_foes
from utility import state_space
from utility import update_parking_reward
from utility import rerouteFoe
from utility import rerouteAgent
from utility import update_traffic_reward
from utility import update_pedestrians_reward
from agent import Agent
from crowdsourcing import Crowdsourcing

import os
import sys
import numpy as np

from sumolib.net import readNet
from sumolib import checkBinary
import traci

import serial
from pynmeagps import NMEAReader

import pyttsx3


net_name = r"unisa_net/osmped.net.xml"
sumocfg_name = r"unisa_net/osmped.sumocfg"
agents_file = r"unisa_net/unisa_agents.npy"
foes_file = r"unisa_net/unisa_foes.npy"
total_cars = 30
parkings = {'-906615585':'biblioteca', '-587489968#0':'terminal', '298563412':'multipiano'}
number_lots = {'-906615585': 1, '-587489968#0':10, '298563412':10}
parking_goals = {0:'-587489968#0', 1:'-906615585',2:'298563412'} # TERMINAL, BIBLIOTECA, MULTIPIANO
termination_steps = 37000
tHor = 5
lay_off = 100000.0 #Very long layoff  

#Loading agent and foe info
agentArray = np.load(agents_file)
foeArray = np.load(foes_file)

#agent info (start, departure time and target)

agDep = [int(x) for x in agentArray[1]]
agGoals = [int(x) for x in agentArray[2]]

#for info (start, departure time and ending edge)
foeSt = foeArray[0]
foeDep = [int(x) for x in foeArray[1]]
foeFin = foeArray[2]

def take_first(elem):
    return elem[0]

def reading_thread(ser):
    global msg
    try:
        nmr = NMEAReader(ser)
        while True:
            (raw_data, msg) = nmr.read() #msg will be global variable that main will read
            # block for a moment
            sleep(0.250)
    except KeyboardInterrupt:
        ser.close()

# this is the main entry point of this script
if __name__ == "__main__":

    new_line = "\n"
    log_gps = open("log_gps.txt", "w")

    ser = serial.Serial('/dev/rfcomm0', 9600)
    
    # create a thread
    thread = Thread(target=reading_thread, args=[ser])
    # run the thread
    thread.start()

    engine = pyttsx3.init()

    """ RATE"""
    engine.setProperty('rate', 140)     # setting up new voice rate
    """VOICE"""
    engine.setProperty('voice', 'english-north')

    n_agents = 1
    n_foes = len(foeSt)
    
    agents = []  
    crowds = []  
    agentHasBeenRerouted = [0]*n_agents #rerouteFlag list that shows if the agent has been rerouted 0-> not been rerouted, 1 rerouted

    # we need to import python modules from the $SUMO_HOME/tools directory
    if 'SUMO_HOME' in os.environ:
        tools= os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variabile 'SUMO_HOME'")

    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoCmd = [sumoBinary, "-c", sumocfg_name,"--step-length","0.05"] #The last parameter is the step size, has to be small
    traci.start(sumoCmd)
    
    print("Starting SUMO...")

    net = readNet(net_name) #read road network
    edge_list = net.getEdges()
    #Dictionary of list IDs
    edge_dict = {}
    for i in range(len(edge_list)):
        edge_dict[i] = edge_list[i].getID()
    blinking_edges = [False]*len(edge_list)

    # --- BLUETOOTH -------------------------------#

    radius = 20
    x, y = net.convertLonLat2XY(msg.lon, msg.lat) 
    #print(f"Latitude: {msg.lat}, Longitude: {msg.lon}")
    now = datetime.now()
    log_gps.write(f"{now} - Latitude: {msg.lat}, Longitude: {msg.lon}{new_line}")
    #x, y = net.convertLonLat2XY(14.788719, 40.776236) # <--------- verify matching edge - lat,lon
    edges = net.getNeighboringEdges(x, y, radius)
    # pick the closest edge

    if len(edges) > 0:
        distancesAndEdges = sorted([(dist, edge) for edge, dist in edges], key=take_first)
        dist, closestEdge = distancesAndEdges[0]
        initial_edge = [closestEdge.getID()]

        agSt = initial_edge
        #add agents and controllers
        for i in range(n_agents):
            agt = Agent(agSt[i], agGoals[i]) 
            agents.append(agt)
            crowds.append(Crowdsourcing(agents[i]))

        #Add agents and foes to SUMO
        add_agents(n_agents,agDep,initial_edge)
        add_foes(n_foes, foeDep, foeSt, foeFin, parkings, lay_off)
        
    # ----------------------------- BLUETOOTH --------#
    
    step=0 #in order to keep track of current time step
    
    #List of each agent's last edge (for now their beginning edge)
    
    past_edge = [traci.vehicle.getRoadID('agent.'+str(i)) for i in range(n_agents)]
    if initial_edge != "392822663" and initial_edge != "392822665#0" and initial_edge != "392822665#1" and initial_edge != "-79262633#16" and initial_edge != "-79262633#14" and initial_edge != "-79262633#8" and initial_edge != "-79262633#3" and initial_edge != "-79262633#0" and initial_edge!= "-587489968#1" and initial_edge!= "587489968#1": #sidewalks
        traci.vehicle.moveToXY("agent.0", initial_edge, 0, x, y) #edges are not sidewalks
        
    else:
        traci.vehicle.moveToXY("agent.0", initial_edge, 1, x, y)
    
    #traci.gui.trackVehicle('View #0', 'agent.0') ---> if you don't want to use the mouse

    #Reward
    pR = update_parking_reward(parkings,edge_list,edge_dict, number_lots)
    tR = update_traffic_reward(edge_list, parkings, step)
    pedR = update_pedestrians_reward(edge_list, parkings, step)
    r = computeReward(pR, tR, pedR) 

    current_edge = initial_edge[0]

    
    while len(getFreeParking(parkings, number_lots)) != 0 and step < termination_steps:

        for i in range(n_agents):
            agentID = "agent." + str(i)
            #for each agent I check if there is in the simulation:
            if agentID in traci.vehicle.getIDList():
                desired_parking = parking_goals[agents[i].id_goal]
                # ----- BLUETOOTH -------------------# 
                x, y = net.convertLonLat2XY(msg.lon, msg.lat) 
                #print(f"Latitude: {msg.lat}, Longitude: {msg.lon}")
                now = datetime.now()
                log_gps.write(f"{now} - Latitude: {msg.lat}, Longitude: {msg.lon}{new_line}")
                #x, y = net.convertLonLat2XY(14.788719, 40.776236) # <------------- verify matching edge - lat,lon
                                                                                                                                                                                             
                if current_edge != "392822668" and current_edge != "392822664#0" and current_edge != "392822663" and current_edge != ":cluster_3960175900_3960175904_1" and current_edge != ":cluster_3960175900_3960175904_4" and current_edge != ":cluster_3960175906_3960175907_2" and current_edge != ":429515568_1": 
                    edges = net.getNeighboringEdges(x, y, radius)
                    if current_edge != "392822663" and current_edge != "392822665#0" and current_edge != "392822665#1" and current_edge != "-79262633#16" and current_edge != "-79262633#14" and current_edge != "-79262633#8" and current_edge != "-79262633#3" and current_edge != "-79262633#0" and current_edge!= "-587489968#1" and current_edge!= "587489968#1":
                        valid = traci.lane.getLinks(current_edge + "_0", False)
                    else:
                        valid = traci.lane.getLinks(current_edge + "_1", False)

                    valid_id = []
                    for ed in valid:
                        ed_id = ed[0].split('_')[0]
                        valid_id.append(ed_id)
                    valid_id.append(current_edge)
                    copy_edges = edges.copy()
                    for ed in copy_edges:
                        if ed[0].getID() not in valid_id:
                            edges.remove(ed)
                    # pick the closest edge
                    
                    if len(edges) > 0 :
                        distancesAndEdges = sorted([(dist, edge) for edge, dist in edges], key=take_first)
                        dist, closestEdge = distancesAndEdges[0]
                        current_edge = closestEdge.getID()
                        if current_edge != "392822663" and current_edge != "392822665#0" and current_edge != "392822665#1" and current_edge != "-79262633#16" and current_edge != "-79262633#14" and current_edge != "-79262633#8" and current_edge != "-79262633#3" and current_edge != "-79262633#0" and current_edge!= "-587489968#1" and current_edge!= "587489968#1":
                            traci.vehicle.moveToXY("agent.0", current_edge, 0, x, y)
                        else:
                            traci.vehicle.moveToXY("agent.0", current_edge, 1, x, y)

                    #traci.gui.trackVehicle('View #0', 'agent.0')
                else: 
                    current_edge = traci.vehicle.getRoadID(agentID)

                if current_edge == "147391538#4" and traci.vehicle.getLanePosition("agent.0") >= 194: #roundabouts
                    current_edge = "392822668"
                    traci.vehicle.moveToXY("agent.0", current_edge, 0, 967.11,2356.07) 
                
                
                if (past_edge[i] == '' or past_edge[i] != current_edge) and current_edge[0] != ":": #: to avoid calculating the route to an intersection
                    state = [x for x in edge_dict if edge_dict[x] == current_edge][0] #get state index 
                    #FIRST CASE: THE PARKING LOT IS NOT FULL, SO THE AGENT CAN STOP
                    if current_edge in parkings and traci.parkingarea.getVehicleCount(parkings[current_edge])< number_lots[current_edge] and current_edge == desired_parking:
                        traci.vehicle.setParkingAreaStop(agentID, parkings[current_edge], duration=lay_off)
                        print(f"Congrats! You reached the parking area {parkings[current_edge]}.")
                        engine.say(f'Congrats! You reached the parking area {parkings[current_edge]}, you will find free spaces to park in')
                        engine.runAndWait()
                        traci.close()
                        ser.close()
                        log_gps.close()
                        exit(0)
                    
                    #OTHER CASES: YOU ARE NOT IN THE PARKING LOT OR THE PARKING LOT IS FULL decision-making loop to find next edge 
                    else: #DM loop
                           
                        new_state = crowds[i].receding_horizon_DM(state, tHor, state_space(state, tHor, edge_dict, net), r)
                        #the new state is the result of crowdsourcing
                        prox_edge = edge_dict[new_state]
                    
                        if blinking_edges[getEdgeIndex(prox_edge, edge_dict)] == False: 
                            change_route_color(prox_edge)
                            blinking_edges[getEdgeIndex(prox_edge, edge_dict)] = True 
                        traci.vehicle.changeTarget(agentID, prox_edge) #the agent will have as new target the state that is just been calculated by crowdsourcing 
                        
                    if past_edge[i] != current_edge and past_edge[i] != '' :
                        change_route_color(current_edge)
                        blinking_edges[getEdgeIndex(current_edge, edge_dict)] = False
                    
                    past_edge[i] = current_edge #Update edge
                    
                        
                    
                #If the desired parking area is full, the agent needs to be rerouted. So we need to assign a new target behavior by calling the function rerouteAgent.

                if (traci.vehicle.isStoppedParking(agentID) == False or traci.vehicle.isStopped(agentID) == False ):
                    if  traci.parkingarea.getVehicleCount(parkings[desired_parking])== number_lots[desired_parking] and agentHasBeenRerouted[i] !=1: #If we need to p rerouted
                        new_parking = rerouteAgent(i, parkings, agents, crowds, number_lots) #in practice, change the target behavior
                        
                        print(f"You have been rerouted towards {parkings[new_parking]} because {parkings[desired_parking]} is full!")
                        engine.say(f'You have been rerouted towards {parkings[new_parking]} because {parkings[desired_parking]} is full!')
                        new_state = crowds[i].receding_horizon_DM(state, tHor, state_space(state, tHor, edge_dict, net), r) #Redo DM loop after rerouting to avoid disappearing 
                        prox_edge = edge_dict[new_state]
                        
                        if blinking_edges[getEdgeIndex(prox_edge, edge_dict)] == False:
                            change_route_color(prox_edge)
                            blinking_edges[getEdgeIndex(prox_edge, edge_dict)] = True 

                        traci.vehicle.changeTarget(agentID, prox_edge)
                        agentHasBeenRerouted[i] = 1
                    if past_edge[i] != current_edge:
                        agentHasBeenRerouted[i] = 0

        for i in range(n_foes): #for each foes
            foeID = 'foe.' + str(i)
            if foeID in traci.vehicle.getIDList(): #if the foe is in the simulation
                foe_edge = traci.vehicle.getRoadID(foeID) #get foe edge
                if traci.vehicle.getRoadID(foeID) == foeFin[i]: 
                    if traci.parkingarea.getVehicleCount(parkings[traci.vehicle.getRoadID(foeID)]) > number_lots[foe_edge] -1: #Reroute if we arrived at a full parking lot
                        rerouteFoe(foeID, parkings, number_lots)
        
        traci.simulationStep() #advancing one time step

        #Reward
        pR = update_parking_reward(parkings,edge_list,edge_dict, number_lots)
        tR = update_traffic_reward(edge_list, parkings, step)
        pedR = update_pedestrians_reward(edge_list, parkings, step)
        r = computeReward(pR, tR, pedR) 

        remaining = total_cars - np.sum([traci.parkingarea.getVehicleCount(parkings[p]) for p in parkings]) #Log unparked cars
        print (f"simulation: {sys.argv[1]} - step: {step}")
        step+=1
    