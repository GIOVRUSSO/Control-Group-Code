from utility import computeReward, getFreeParking, get_options
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


net_name = r"unisa_net/osmped.net.xml"
sumocfg_name = r"unisa_net/osmped.sumocfg"
agents_file = r"unisa_net/unisa_agents.npy"
foes_file = r"unisa_net/unisa_foes.npy"
total_cars = 200
parkings = {'-906615585':'biblioteca', '-587489968#0':'terminal', '298563412':'multipiano'}
number_lots = {'-906615585': 66, '-587489968#0':66, '298563412':68}
parking_goals = {0:'-587489968#0', 1:'-906615585',2:'298563412'} # TERMINAL, BIBLIOTECA, MULTIPIANO
parking_coordinates = {'-906615585':[923.02,2090.34], '-587489968#0':[1207.23,2122.00], '298563412':[1417.67,1367.76]}

termination_steps = 40000

# UNISA logs

parked_agents_file = r"unisa_net/logs/unisa_parked_agents"
parked_foes_file = r"unisa_net/logs/unisa_parked_foes"
unparked_cars_file = r"unisa_net/logs/unisa_unparked"

speed_meanagents_file = r"unisa_net/logs/unisa_speed_agents"
co2_agents_file = r"unisa_net/logs/unisa_co2_agents"
co_agents_file = r"unisa_net/logs/unisa_co_agents"
traveltime_agents_file = r"unisa_net/logs/unisa_traveltime_agents"
fuel_agents_file = r"unisa_net/logs/unisa_fuel_agents"
noise_agents_file= r"unisa_net/logs/unisa_noise_agents"

co2_meanagents_file = r"unisa_net/logs/unisa_co2_agentsmean"
co_meanagents_file = r"unisa_net/logs/unisa_co_agentsmean"
fuel_meanagents_file = r"unisa_net/logs/unisa_fuel_agentsmean"
noise_meanagents_file= r"unisa_net/logs/unisa_noise_agentsmean"

speed_meanfoes_file = r"unisa_net/logs/unisa_speed_foes"
co2_foes_file = r"unisa_net/logs/unisa_co2_foes"
co_foes_file = r"unisa_net/logs/unisa_co_foes"
traveltime_foes_file = r"unisa_net/logs/unisa_traveltime_foes"
fuel_foes_file = r"unisa_net/logs/unisa_fuel_foes"
noise_foes_file= r"unisa_net/logs/unisa_noise_foes"

co2_meanfoes_file = r"unisa_net/logs/unisa_co2_foesmean"
co_meanfoes_file = r"unisa_net/logs/unisa_co_foesmean"
fuel_meanfoes_file = r"unisa_net/logs/unisa_fuel_foesmean"
noise_meanfoes_file= r"unisa_net/logs/unisa_noise_foesmean"

tHor = 5
lay_off = 100000.0 #Very long layoff  


#Initializing log lists
logs = []
co2_agents = []
noise_agents = []
co_agents = []
fuel_agents = []
co2_foes = []
co_foes = []
fuel_foes = []
noise_foes = []
np_foes = []
np_agents = []

#Loading agent and foe info
agentArray = np.load(agents_file)
foeArray = np.load(foes_file)

#agent info (start, departure time and target)
agSt = agentArray[0]
agDep = [int(x) for x in agentArray[1]]
agGoals = [int(x) for x in agentArray[2]]

#for info (start, departure time and ending edge)
foeSt = foeArray[0]
foeDep = [int(x) for x in foeArray[1]]
foeFin = foeArray[2]

# this is the main entry point of this script
if __name__ == "__main__":
    
    n_agents = len(agSt)
    n_foes = len(foeSt)

    speed_meanagents = [ [] for _ in range(n_agents) ]
    speed_meanfoes= [ [] for _ in range(n_foes) ]

    co2_meanfoes = [ [] for _ in range(n_foes) ]
    co_meanfoes = [ [] for _ in range(n_foes) ]
    fuel_meanfoes = [ [] for _ in range(n_foes) ]
    noise_meanfoes = [ [] for _ in range(n_foes) ]

    co2_meanagents = [ [] for _ in range(n_agents) ]
    co_meanagents = [ [] for _ in range(n_agents) ]
    fuel_meanagents = [ [] for _ in range(n_agents) ]
    noise_meanagents= [ [] for _ in range(n_agents) ]

    
    agents = []  #structure that maintains the instances of agents
    crowds = []  #structure that keeps instances to the crowdsoucing class of agents
    agentHasBeenRerouted = [0]*n_agents #rerouteFlag list that shows if the agent has been rerouted 0-> not been rerouted, 1 rerouted
    
    #add agents and controllers
    for i in range(n_agents):
        agt = Agent(agSt[i], agGoals[i])
        agents.append(agt)
        crowds.append(Crowdsourcing(agents[i]))

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
    #Dicionnary of list IDs
    edge_dict = {}
    for i in range(len(edge_list)):
        edge_dict[i] = edge_list[i].getID()

    #Add agents and foes to SUMO
    add_agents(n_agents,agDep,agSt)
    add_foes(n_foes, foeDep, foeSt, foeFin, parkings, lay_off)
  
    
    step=0 #in order to keep track of current time step
    
    #List of each agent's last edge (for now their beginning edge)
    past_edge = [traci.vehicle.getRoadID('agent.'+str(i)) for i in range(n_agents)]
    
    #Reward
    pR = update_parking_reward(parkings,edge_list,edge_dict, number_lots)
    tR = update_traffic_reward(edge_list, parkings, step)
    pedR = update_pedestrians_reward(edge_list, parkings, step)
    r = computeReward(pR, tR, pedR) 

    traveltimeA = [0]*n_agents
    traveltimeF = [0]*n_foes

    while len(getFreeParking(parkings, number_lots)) != 0 and step < termination_steps:
        co2_agents.append(0)
        co_agents.append(0)
        fuel_agents.append(0)
        noise_agents.append(0)
        co2_foes.append(0)
        co_foes.append(0)
        fuel_foes.append(0)
        noise_foes.append(0)
        np_agents.append(0)
        np_foes.append(0)

        agentsMap = 0
        foesMap = 0

        for i in range(n_agents):
            agentID = "agent." + str(i)
            #for each agent I check if there is in the simulation:
            if agentID in traci.vehicle.getIDList():
                agentsMap += 1
                desired_parking = parking_goals[agents[i].id_goal]
                current_edge = traci.vehicle.getRoadID(agentID) #get current edge
                
                if traci.vehicle.isStoppedParking(agentID) == False: #take evaluation metrics if the car is not parked
                    speed_meanagents[i].append(traci.vehicle.getSpeed(agentID))  #Speed profile logging for each agent
                    co2_meanagents[i].append(traci.vehicle.getCO2Emission(agentID))
                    co_meanagents[i].append(traci.vehicle.getCOEmission(agentID))
                    fuel_meanagents[i].append(traci.vehicle.getFuelConsumption(agentID))
                    noise_meanagents[i].append(traci.vehicle.getNoiseEmission(agentID))
                

                co2_agents[step] += traci.vehicle.getCO2Emission(agentID)
                co_agents[step] += traci.vehicle.getCOEmission(agentID)
                fuel_agents[step] += traci.vehicle.getFuelConsumption(agentID)
                noise_agents[step] += traci.vehicle.getNoiseEmission(agentID)
                
                if (past_edge[i] == '' or past_edge[i] != current_edge) and current_edge[0] != ":":  #: to avoid calculating the route to an intersection
                    state = [x for x in edge_dict if edge_dict[x] == current_edge][0] #get state index
                    
                    #FIRST CASE: THE PARKING LOT IS NOT FULL, SO THE AGENT CAN STOP
                    if current_edge in parkings and traci.parkingarea.getVehicleCount(parkings[current_edge])< number_lots[current_edge] and current_edge == desired_parking:
                        traci.vehicle.setParkingAreaStop(agentID, parkings[current_edge], duration=lay_off)
                        traveltimeA[i] = traci.vehicle.getTimeLoss(agentID)
                    
                    #OTHER CASES: YOU ARE NOT IN THE PARKING LOT OR THE PARKING LOT IS FULL -> decision-making loop to find next edge 
                    else: #DM loop
                        new_state = crowds[i].receding_horizon_DM(state, tHor, state_space(state, tHor, edge_dict, net), r)
                        #the new state is the result of crowdsourcing
                        prox_edge = edge_dict[new_state]
                    
                        traci.vehicle.changeTarget(agentID, prox_edge) #the agent will have as new target the state that is just been calculated by crowdsourcing 
                        
                    past_edge[i] = current_edge #Update edge
                
                #If the desired parking area is full, the agent needs to be rerouted. So we need to assign a new target behavior by calling the function rerouteAgent.
                if (traci.vehicle.isStoppedParking(agentID) == False or traci.vehicle.isStopped(agentID) == False ):
                    if  traci.parkingarea.getVehicleCount(parkings[desired_parking])== number_lots[desired_parking] and agentHasBeenRerouted[i] !=1:
                        new_parking = rerouteAgent(i, parkings, agents, crowds, number_lots, parking_goals, parking_coordinates) #in practice, change the target behavior
                        print(f"{agentID} has been rerouted towards {parkings[new_parking]} because {parkings[desired_parking]} is full!")
                        new_statee = crowds[i].receding_horizon_DM(state, tHor, state_space(state, tHor, edge_dict, net), r) #Redo DM loop after rerouting
                        prox_edge = edge_dict[new_statee]
                        traci.vehicle.changeTarget(agentID, prox_edge)
                        agentHasBeenRerouted[i] = 1
                    if past_edge[i] != current_edge:
                        agentHasBeenRerouted[i] = 0

        if agentsMap!=0:
            co2_agents[step] /= agentsMap
            co_agents[step] /= agentsMap
            fuel_agents[step] /= agentsMap
            noise_agents[step] /= agentsMap

        if step % 10 ==0 and n_agents >0:
            np.save(speed_meanagents_file +str(sys.argv[1])+'.npy',np.array(speed_meanagents))
            np.save(co2_agents_file +str(sys.argv[1])+'.npy',np.array(co2_agents))
            np.save(co_agents_file +str(sys.argv[1])+'.npy',np.array(co_agents))
            np.save(traveltime_agents_file +str(sys.argv[1])+'.npy',np.array(traveltimeA))
            np.save(fuel_agents_file +str(sys.argv[1])+'.npy',np.array(fuel_agents))
            np.save(noise_agents_file +str(sys.argv[1])+'.npy',np.array(noise_agents))
            np.save(co2_meanagents_file +str(sys.argv[1])+'.npy',np.array(co2_meanagents))
            np.save(co_meanagents_file +str(sys.argv[1])+'.npy',np.array(co_meanagents))
            np.save(fuel_meanagents_file +str(sys.argv[1])+'.npy',np.array(fuel_meanagents))
            np.save(noise_meanagents_file +str(sys.argv[1])+'.npy',np.array(noise_meanagents))


        for i in range(n_foes): #for each foes
            foeID = 'foe.' + str(i)
            if foeID in traci.vehicle.getIDList(): #if the foe is in the simulation

                if traci.vehicle.isStoppedParking(foeID) == False:
                    speed_meanfoes[i].append(traci.vehicle.getSpeed(foeID))  #Speed profile logging for each foe
                    co2_meanfoes[i].append(traci.vehicle.getCO2Emission(foeID))
                    co_meanfoes[i].append(traci.vehicle.getCOEmission(foeID))
                    fuel_meanfoes[i].append(traci.vehicle.getFuelConsumption(foeID))
                    noise_meanfoes[i].append(traci.vehicle.getNoiseEmission(foeID))

                co2_foes[step] += traci.vehicle.getCO2Emission(foeID)
                co_foes[step] += traci.vehicle.getCOEmission(foeID)
                fuel_foes[step] += traci.vehicle.getFuelConsumption(foeID)
                noise_foes[step] += traci.vehicle.getNoiseEmission(foeID)


                foe_edge = traci.vehicle.getRoadID(foeID) #get current edge
                foesMap = foesMap + 1 #counter of the foes

                if traci.vehicle.getRoadID(foeID) == foeFin[i]: 
                    if traci.parkingarea.getVehicleCount(parkings[traci.vehicle.getRoadID(foeID)]) > number_lots[foe_edge] -1: #Reroute if we arrived at a full parking lot
                        rerouteFoe(foeID, parkings, number_lots)

                if traci.vehicle.isStoppedParking(foeID) and traveltimeF[i] == 0:
                    traveltimeF[i] = traci.vehicle.getTimeLoss(foeID)

        if (foesMap !=0):
            co2_foes[step] /= foesMap
            co_foes[step] /= foesMap
            fuel_foes[step] /= foesMap
            noise_foes[step] /= foesMap

        if step % 10 ==0 and n_foes >0:
            np.save(co2_foes_file +str(sys.argv[1])+'.npy',np.array(co2_foes))
            np.save(co_foes_file +str(sys.argv[1])+'.npy',np.array(co_foes))
            np.save(traveltime_foes_file +str(sys.argv[1])+'.npy',np.array(traveltimeF))
            np.save(fuel_foes_file +str(sys.argv[1])+'.npy',np.array(fuel_foes))
            np.save(noise_foes_file +str(sys.argv[1])+'.npy',np.array(noise_foes))
            np.save(speed_meanfoes_file +str(sys.argv[1])+'.npy',np.array(speed_meanfoes))
            np.save(co2_meanfoes_file +str(sys.argv[1])+'.npy',np.array(co2_meanfoes))
            np.save(co_meanfoes_file +str(sys.argv[1])+'.npy',np.array(co_meanfoes))
            np.save(fuel_meanfoes_file +str(sys.argv[1])+'.npy',np.array(fuel_meanfoes))
            np.save(noise_meanfoes_file +str(sys.argv[1])+'.npy',np.array(noise_meanfoes))

        traci.simulationStep() #advancing one time step


        #Reward
        pR = update_parking_reward(parkings,edge_list,edge_dict, number_lots)
        tR = update_traffic_reward(edge_list, parkings, step)
        pedR = update_pedestrians_reward(edge_list, parkings, step)
        r = computeReward(pR, tR, pedR) 

        remaining = total_cars - np.sum([traci.parkingarea.getVehicleCount(parkings[p]) for p in parkings]) #Log unparked cars
        logs.append(remaining)
        print (f"simulation: {sys.argv[1]} - step: {step} - remaining: {remaining}")

        for p in parkings:
            parked_vehicles = traci.parkingarea.getVehicleIDs(parkings[p])
            for v in parked_vehicles:
                if v.startswith("agent"):
                    np_agents[step] +=1
                elif v.startswith("foe"): 
                    np_foes[step] +=1

        if step % 10 == 0:
            np.save(unparked_cars_file +str(sys.argv[1])+'.npy',np.array(logs)) 
            if n_foes>0:
                np.save(parked_foes_file +str(sys.argv[1])+'.npy',np.array(np_foes))
            if n_agents>0:    
                np.save(parked_agents_file +str(sys.argv[1])+'.npy',np.array(np_agents))

        step+=1

        

    #Save results

    np.save(unparked_cars_file +str(sys.argv[1])+'.npy',np.array(logs)) 

    if n_foes >0:
        np.save(parked_foes_file +str(sys.argv[1])+'.npy',np.array(np_foes))
        np.save(speed_meanfoes_file +str(sys.argv[1])+'.npy',np.array(speed_meanfoes))
        np.save(co2_foes_file +str(sys.argv[1])+'.npy',np.array(co2_foes))
        np.save(co_foes_file +str(sys.argv[1])+'.npy',np.array(co_foes))
        np.save(traveltime_foes_file +str(sys.argv[1])+'.npy',np.array(traveltimeF))
        np.save(fuel_foes_file +str(sys.argv[1])+'.npy',np.array(fuel_foes))
        np.save(noise_foes_file +str(sys.argv[1])+'.npy',np.array(noise_foes))

        np.save(co2_meanfoes_file +str(sys.argv[1])+'.npy',np.array(co2_meanfoes))
        np.save(co_meanfoes_file +str(sys.argv[1])+'.npy',np.array(co_meanfoes))
        np.save(fuel_meanfoes_file +str(sys.argv[1])+'.npy',np.array(fuel_meanfoes))
        np.save(noise_meanfoes_file +str(sys.argv[1])+'.npy',np.array(noise_meanfoes))

    if n_agents >0:
        np.save(parked_agents_file +str(sys.argv[1])+'.npy',np.array(np_agents))
        np.save(speed_meanagents_file +str(sys.argv[1])+'.npy',np.array(speed_meanagents))
        np.save(co2_agents_file +str(sys.argv[1])+'.npy',np.array(co2_agents))
        np.save(co_agents_file +str(sys.argv[1])+'.npy',np.array(co_agents))
        np.save(traveltime_agents_file +str(sys.argv[1])+'.npy',np.array(traveltimeA))
        np.save(fuel_agents_file +str(sys.argv[1])+'.npy',np.array(fuel_agents))
        np.save(noise_agents_file +str(sys.argv[1])+'.npy',np.array(noise_agents))
        np.save(co2_meanagents_file +str(sys.argv[1])+'.npy',np.array(co2_meanagents))
        np.save(co_meanagents_file +str(sys.argv[1])+'.npy',np.array(co_meanagents))
        np.save(fuel_meanagents_file +str(sys.argv[1])+'.npy',np.array(fuel_meanagents))
        np.save(noise_meanagents_file +str(sys.argv[1])+'.npy',np.array(noise_meanagents))
    
    traci.close()
