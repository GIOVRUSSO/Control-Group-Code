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

#SALERNO NET ------------------------------------------------------------------------
net_name = r"salerno_net/osm.net.xml"
net_namecars = r"salerno_net/osmcars.net.xml"
sumocfg_name = r"salerno_net/osm.sumocfg"
agents_file = r"salerno_net/salerno_agents.npy"
foes_file = r"salerno_net/salerno_foes.npy"
total_cars = 200

parkings = {'672732730':'p_molo_manfredi', '-672732731':'p_molo_manfredi2',
 '-672732728':'p_molo_manfredi3', '401420254#0':'p_molo_manfredi4',
 '401420254#4':'p_molo_manfredi5', '-1001198066': 'p_liberta',
 '744932253':'p_stabia', '670934106':'p_amendola', '670934108': 'p_amendola2',
 '160821659#2': 'p_concordia', '671983008':'p_concordia2', '92961457#3': 'p_concordia3',
 '92961457#4': 'p_concordia4', '92961462':'p_concordia5', '672273418#3':'p_concordia6',
 '673737658#3':'p_mazzini', '673737658#6':'p_mazzini2', '-565381618#0':'p_stazione',
 '163563222':'p_foceirno', 'E5':'p_vinciprova', 'E6':'p_vinciprova2' }

number_lots = {'672732730':9, '-672732731':9, '-672732728':9, '401420254#0':9, '401420254#4':9, 
                '-1001198066':9, '744932253':9, '670934106':9, '670934108' :9, 
                '160821659#2':9, '671983008':9, '92961457#3':9, '92961457#4':9, '92961462':9,
                '672273418#3':9, '673737658#3':9, '673737658#6':9, '-565381618#0':9, '163563222':9, 'E5':9, 'E6':9}

parking_goals = {0:'672732730', 1:'-672732731',2:'-672732728', 3: '401420254#0', 4: '401420254#4', 
                    5: '-1001198066', 6: '744932253', 7: '670934106', 8: '670934108', 9: '160821659#2', 
                    10: '671983008', 11: '92961457#3', 12: '92961457#4', 13: '92961462', 14: '672273418#3',
                    15: '673737658#3', 16: '673737658#6', 17: '-565381618#0', 18:'163563222', 19: 'E5', 20: 'E6'}

parking_coordinates = {'672732730':[5617.37,1300.41], '-672732731':[5635.93,1252.87],
 '-672732728':[5659.54,1289.71], '401420254#0':[5629.66,1243.59],
 '401420254#4':[5602.24,1321.00], '-1001198066': [5719.16,1318.55],
 '744932253':[5711.21,1394.70], '670934106':[5995.94,1598.32], '670934108': [5948.26,1588.53],
 '160821659#2': [7154.11,972.20], '671983008':[7110.46,1098.23], '92961457#3': [7183.04,1049.94],
 '92961457#4': [7136.29,1052.83], '92961462':[7214.83,1050.77], '672273418#3':[7262.64,992.02],
 '673737658#3':[7227.06,1113.90], '673737658#6':[7236.69,1138.24], '-565381618#0':[7315.48,1499.06],
 '163563222':[7581.81,1026.51], 'E5':[7646.73,1166.86], 'E6': [7737.35,1123.43] }

termination_steps = 40000

# logs
parked_agents_file = r"salerno_net/logs/salerno_parked_agents"
parked_foes_file = r"salerno_net/logs/salerno_parked_foes"
unparked_cars_file = r"salerno_net/logs/salerno_unparked"

speed_meanagents_file = r"salerno_net/logs/salerno_speed_agents"
co2_agents_file = r"salerno_net/logs/salerno_co2_agents"
co_agents_file = r"salerno_net/logs/salerno_co_agents"
traveltime_agents_file = r"salerno_net/logs/salerno_traveltime_agents"
fuel_agents_file = r"salerno_net/logs/salerno_fuel_agents"
noise_agents_file= r"salerno_net/logs/salerno_noise_agents"

co2_meanagents_file = r"salerno_net/logs/salerno_co2_agentsmean"
co_meanagents_file = r"salerno_net/logs/salerno_co_agentsmean"
fuel_meanagents_file = r"salerno_net/logs/salerno_fuel_agentsmean"
noise_meanagents_file= r"salerno_net/logs/salerno_noise_agentsmean"

speed_meanfoes_file = r"salerno_net/logs/salerno_speed_foes"
co2_foes_file = r"salerno_net/logs/salerno_co2_foes"
co_foes_file = r"salerno_net/logs/salerno_co_foes"
traveltime_foes_file = r"salerno_net/logs/salerno_traveltime_foes"
fuel_foes_file = r"salerno_net/logs/salerno_fuel_foes"
noise_foes_file= r"salerno_net/logs/salerno_noise_foes"

co2_meanfoes_file = r"salerno_net/logs/salerno_co2_foesmean"
co_meanfoes_file = r"salerno_net/logs/salerno_co_foesmean"
fuel_meanfoes_file = r"salerno_net/logs/salerno_fuel_foesmean"
noise_meanfoes_file= r"salerno_net/logs/salerno_noise_foesmean"


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
    sumoCmd = [sumoBinary, "-c", sumocfg_name,"--step-length","0.05"] #   The last parameter is the step size, has to be small
    traci.start(sumoCmd)
    
    print("Starting SUMO...")
    
    net = readNet(net_name) #read road network
    edge_list = net.getEdges()
    #Dictionary of list IDs
    edge_dict = {}
    for i in range(len(edge_list)):
        edge_dict[i] = edge_list[i].getID()

    netcars = readNet(net_namecars)
    edge_listcars = netcars.getEdges()
    #Dictionary of list IDs
    edge_dictcars = {}

    for i in range(len(edge_listcars)):
        edge_dictcars[i] = edge_listcars[i].getID()

    #Add agents and foes to SUMO
    add_agents(n_agents,agDep,agSt)
    add_foes(n_foes, foeDep, foeSt, foeFin, parkings, lay_off)
    
    remaining = total_cars
    step=0 #in order to keep track of current time step
    
    #List of each agent's last edge (for now their beginning edge)

    past_edge = [traci.vehicle.getRoadID('agent.'+str(i)) for i in range(n_agents)]
    
    #Reward
    pR = update_parking_reward(parkings,edge_listcars,edge_dictcars, number_lots)
    tR = update_traffic_reward(edge_listcars, parkings, step)
    pedR = update_pedestrians_reward(edge_listcars, parkings, step)
    
    r = computeReward(pR, tR, pedR) 

    traveltimeA = [0]*n_agents
    traveltimeF = [0]*n_foes

    while len(getFreeParking(parkings, number_lots)) != 0 and  step < termination_steps: 

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
                
                if traci.vehicle.isStoppedParking(agentID) == False: #get evaluation metrics if it is not stopped at a parking lot
                    speed_meanagents[i].append(traci.vehicle.getSpeed(agentID))  
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
                    
                    statecars = [x for x in edge_dictcars if edge_dictcars[x] == current_edge][0] #get state index
                                        
                    #FIRST CASE: THE PARKING LOT IS NOT FULL, SO THE AGENT CAN STOP
                    if current_edge in parkings and traci.parkingarea.getVehicleCount(parkings[current_edge])< number_lots[current_edge] and current_edge == desired_parking: 
                        traci.vehicle.setParkingAreaStop(agentID, parkings[current_edge], duration=lay_off)
                        traveltimeA[i] = traci.vehicle.getTimeLoss(agentID) #get the time spent from when it is spawned into the simulation to when it parks.
                    
                    #OTHER CASES: YOU ARE NOT IN THE PARKING LOT OR THE PARKING LOT IS FULL -> decision-making loop to find next edge 
                    else: #DM loop

                        laneID = traci.vehicle.getLaneID(agentID)
                        successors = traci.lane.getLinks(laneID, False)
       
                        if len(successors) >1 or current_edge in ["50702263#3", "23653449#17", "23653449#18", "50733021#0", "671471170#2", "671188880#1", "50695151#1", "50695151#4", "677091106", "398058811", "50701873#0", "50701873#1", "766350959#2", "766511482#1", "668814536#0", "50701610#8", "531401290#0", "50706083#4", "93713834#0", "50793376#0", "50733593", "398055968#3", "50731850#0", "185483089#2", "185483089#5", "185483089#7", "185483089#14", "185483089#16", "185483089#20", "50733021#2", "766511481", "668814536#3"]:
                            new_state = crowds[i].receding_horizon_DM(statecars, tHor, state_space(statecars, tHor, edge_dictcars, netcars), r)
                            #the new state is the result of crowdsourcing if there is more than one successor or there are multiple lanes.
                            prox_edge = edge_dictcars[new_state]
                        
                        else: #the only successor will be the prox edge
                            prox_edge = successors[0][0].split('_')[0]
                    
                        traci.vehicle.changeTarget(agentID, prox_edge) #the agent will have as new target the state that is just been calculated by crowdsourcing   
                
                    
                    past_edge[i] = current_edge #Update edge
                
                #If the desired parking area is full, the agent needs to be rerouted. So we need to assign a new target behavior by calling the function rerouteAgent.
                if (traci.vehicle.isStoppedParking(agentID) == False or traci.vehicle.isStopped(agentID) == False ):
                    if  traci.parkingarea.getVehicleCount(parkings[desired_parking])== number_lots[desired_parking] and agentHasBeenRerouted[i] !=1: #If we need to p rerouted
                        new_parking = rerouteAgent(i, parkings, agents, crowds, number_lots, parking_goals, parking_coordinates) #in practice, change the target behavior
                        print(f"{agentID} has been rerouted towards {parkings[new_parking]} because {parkings[desired_parking]} is full!")
                        new_statee = crowds[i].receding_horizon_DM(statecars, tHor, state_space(statecars, tHor, edge_dictcars, netcars), r) #Redo DM loop after rerouting to avoid disappearing 
                        prox_edge = edge_dictcars[new_statee]
                        traci.vehicle.changeTarget(agentID, prox_edge)
                        agentHasBeenRerouted[i] = 1
                    if past_edge[i] != current_edge:
                        agentHasBeenRerouted[i] = 0
                

        if agentsMap!=0:
            co2_agents[step] /= agentsMap
            co_agents[step] /= agentsMap
            fuel_agents[step] /= agentsMap
            noise_agents[step] /= agentsMap
        
        if step % 10 ==0:
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
                    speed_meanfoes[i].append(traci.vehicle.getSpeed(foeID))  
                    co2_meanfoes[i].append(traci.vehicle.getCO2Emission(foeID))
                    co_meanfoes[i].append(traci.vehicle.getCOEmission(foeID))
                    fuel_meanfoes[i].append(traci.vehicle.getFuelConsumption(foeID))
                    noise_meanfoes[i].append(traci.vehicle.getNoiseEmission(foeID))


                co2_foes[step] += traci.vehicle.getCO2Emission(foeID)
                co_foes[step] += traci.vehicle.getCOEmission(foeID)
                fuel_foes[step] += traci.vehicle.getFuelConsumption(foeID)
                noise_foes[step] += traci.vehicle.getNoiseEmission(foeID)

                foe_edge = traci.vehicle.getRoadID(foeID) #get foe edge
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

        if step % 10 ==0:
            
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

        pR = update_parking_reward(parkings,edge_listcars,edge_dictcars, number_lots)
        tR = update_traffic_reward(edge_listcars, parkings, step)
        pedR = update_pedestrians_reward(edge_listcars, parkings, step)
        r = computeReward(pR, tR, pedR) 

        remaining = total_cars - np.sum([traci.parkingarea.getVehicleCount(parkings[p]) for p in parkings]) #Log unparked cars
        logs.append(remaining)
        print (f"simulation: {sys.argv[1]} - step: {step} - remaining: {remaining}")

        for p in parkings: #counting controlled and uncontrolled cars
            parked_vehicles = traci.parkingarea.getVehicleIDs(parkings[p])
            for v in parked_vehicles:
                if v.startswith("agent"):
                    np_agents[step] +=1
                elif v.startswith("foe"): 
                    np_foes[step] +=1

        if step % 10 == 0:
            np.save(unparked_cars_file +str(sys.argv[1])+'.npy',np.array(logs)) 
            np.save(parked_foes_file +str(sys.argv[1])+'.npy',np.array(np_foes))
            np.save(parked_agents_file +str(sys.argv[1])+'.npy',np.array(np_agents))

        step+=1

        

    #Save results
    np.save(unparked_cars_file +str(sys.argv[1])+'.npy',np.array(logs)) 
    np.save(parked_foes_file +str(sys.argv[1])+'.npy',np.array(np_foes))
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

    traci.close()
    
