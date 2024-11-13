# Module for running simulations for algorithm validation

from vehicledata import VehicleData
import random
import traci
import numpy as np
from bus_parser import bus_parser
from behaviours_maker import valid_neighbors
from behaviours_maker import online_create_behaviours
from intercar_comm import tweet
from spawners import *
from colors import *
from text2speech_handler import playAudio
from text2speech_handler import search_next
import time

WORKS_THRESHOLD = 1 # threshold value after which an area is considered to be "work-in-progress"
PLAY_AUDIO = True # flag for defining whether to play direction audios or not

# method for making a car signal a fake wip area, inputs are:
# - probability of signalling the edge as a wip area,
# - vehicle emitting the signal (instance of VehicleData),
# - id of the edge on which the vehicle is,
# - list of wip areas
def randomSignal(prob,vehicle,roadid,works):
    signal = random.random()<prob
    if signal: # according to probability, launch signal
        if roadid in works: # vehicle correctly signalled a wip area, double its influence
            vehicle.influence = vehicle.influence*2
            if vehicle.influence>1:
                vehicle.influence = 1
        else: # vehicle emitted a fake signal, halve its influence
            vehicle.influence = vehicle.influence/2

# method for building reduced state space for crowdsourcing algorithm, starting from a node, inputs are:
# - index of edge from which the state space must be built,
# - time horizon for crowdsourcing algorithm, in this case the number of crossings to consider for neighbouring edges,
# - data structure containing all static data related to the map,
# - target edge of the vehicle calling the method
def state_space(n_node, tHor,mapdata,target):
    edgelist = mapdata.edgelist
    stsp = [n_node]
    for t in range(tHor): # iteratively look for neighbours for a number of crossings equal to the time horizon
        newSpace = stsp.copy()
        for n in stsp:
            ed = edgelist[n]
            for x in valid_neighbors(ed,mapdata,target,checkIngoing=True):
                xid = edgelist.index(x)
                if xid not in newSpace:
                    newSpace.append(xid) # and add them to the return value
        stsp = newSpace
    return(np.array(stsp))

# method for creating the reward array for each of the edges of the reduced state space, inputs are:
# - list of edges belonging to the reduced state space,
# - data structure containing all static data related to the map,
# - target edge of the vehicle calling this method,
# - list of known wip areas,
# - instance of VehicleData related to the car calling this function,
# - data structure containing all paths,
# - data structure containing all agents
def compute_reward(ss,mapdata,works,vehicle,paths,agents):
    worksweight = 2000 # weight to assign to wip areas
    edgelist = mapdata.edgelist
    net = mapdata.net
    fullen = len(edgelist)
    r = [0]*fullen
    worktolookfor = None
    checkfortail = False if edgelist[ss[0]].getID() not in paths else any(x in paths[edgelist[ss[0]].getID()][agents[vehicle.id].targetIndex] for x in works)
    target_path = paths[edgelist[ss[0]].getID()][agents[vehicle.id].targetIndex]
    for p in target_path:
        if p in works:
            worktolookfor = p
            break
    for i in ss:
        edge = edgelist[i]
        edid = edge.getID()
        veh_list = traci.edge.getIDList()
        busnum = 0
        for id in veh_list:
            if id.__contains__('bus'):
                busnum += 1
        # calculate density of the edge as the length of all vehicle on the edge divided by the edge length in meters
        veh_num = traci.edge.getLastStepVehicleNumber(edid)
        carnum = veh_num-busnum
        density = (5*carnum+7*busnum)/net.getEdge(edid).getLength()
        if density>1:
            density = 1
        road_in_works = worksweight if edid in works else 0 # add a cost to wip areas
        # calculate cost related to density multiplied by the difference between max speed and actual speed
        lastspeed = traci.edge.getLastStepMeanSpeed(edid)
        road_max_speed = traci.lane.getMaxSpeed(net.getEdge(edid).getLanes()[0].getID())
        speed_dif = road_max_speed-lastspeed
        roadspeed = 0 if speed_dif<0 else speed_dif
        # calculate cost related to slowdown tail
        relevance = 0
        for pindex in range(4):
            if edid in paths[edgelist[ss[0]].getID()][agents[vehicle.id].targetIndex+pindex]:
                relevance += 0.5
        if relevance == 0:
            relevance = 0.4
        checkfortail = False if edid not in paths else any(x in paths[edid][agents[vehicle.id].targetIndex] for x in works)
        worktail = 20
        path = paths[edid][agents[vehicle.id].targetIndex]
        if checkfortail and worktolookfor is not None:
            distance = 0
            weight = 1000
            for p in path:
                distance += 1
                if p == worktolookfor:
                    break
                elif p in works:
                    weight = 1000/relevance
                    break
                if distance >= 20:
                    break
            worktail = weight/(5 if distance<20 else 20)/(1 if relevance!=0.4 or distance <20 else 0.4)
        
        r[i] = -road_in_works-roadspeed*density-worktail # calculate and assign reward
        
        print('reward edge '+str(edid)+': '+str(-road_in_works)+' +(-('+str(road_max_speed)+'-'+str(lastspeed)+')*'+str(density)+'='+str(-roadspeed*density)+') + '+str(-worktail)+' = '+str(r[i]))
        
    return np.array(r)

# method to convert items of the state space array into edges, inputs are:
# - array of indexes representing the reduced state space,
# - list of traversable edges in the map
def conv_ss2edges(state_space,edgelist):
    ss = []
    for s in state_space:
        edind = edgelist[s]
        ss.append(edind)
    return ss

# method to update wip areas according to collisions (THIS WILL PROBABLY BE ELIMINATED, IT CAUSES PROBLEMS WITH THE COST FUNCTION), inputs are:
# - list of wip areas
def update_works(works):
    coll_list = traci.simulation.getCollidingVehiclesIDList()
    ret_works = []
    for l in coll_list:
        id = traci.vehicle.getRoadID(l)
        if id not in works and id not in ret_works and len(id)>1:
            ret_works.append(id)
    return ret_works

# method to load paths from file, inputs are:
# - scenario name,
# - number of behaviours (number of paths for each target),
# - flag to establish whether to load paths for scenarios including wip areas or not
def load_offline_paths(scenario,num_algs,consider_works):
    data_structure = np.load('paths_'+str(scenario)+'_'+str(num_algs)+('_wip' if consider_works else '')+'.npy',allow_pickle=True)
    paths = {}
    for i in data_structure.item():
        paths[i] = data_structure.item()[i]
    return paths
    
MEASURING = False # set to True if you want to measure time for executing the algorithm

# method for running a single simulation (in this case, vehicle-in-the-loop) and gathering data, inputs are:
# - number of vehicles involved in the simulation,
# - percentage of cars traversing paths of interest,
# - flag to either use SUMO GUI or not,
# - time horizon for the crowdsourcing algorithm,
# - step size for the simulations,
# - flag to establish whether buses can appear in the simulation or not,
# - flag to establish whether cars with random paths can appear in the simulation or not,
# - hour of the day at which the simulation starts (useful for buses and their scheduling),
# - percentage of agent cars among the ones traversing paths of interest,
# - data structure containing all static data related to the map,
# - flag to establish whether to use a fixed number of agents or not (if False, the number of agents is calculated according to the percentage)(optional),
# - number of desired agents (optional),
# - number of behaviours to take into account (optional),
# - flag to establish whether behaviours should be built online or not (if False, the respective "behaviours" npy file must be loaded)(optional),
# - flag to establish whether wip areas must be included in the simulation or not (useful for switching between scenarios)(optional)
def single_sim(NUM_VEHICLES, PERC_UNI_CARS, SHOW_GUI, T_HORIZON, STEP_SIZE, INCLUDE_BUS, INCLUDE_RANDOM, START_TIME, PERC_AGENT_CARS, mapdata, USE_DESIRED_AGENTS=False, DESIRED_AGENTS=0, NUM_ALGS=1, ONLINE=False, CONSIDER_WORKS=False):
    LANG = 'it'
    SCENARIO = mapdata.scenario
    PERC_AGENT_CARS = DESIRED_AGENTS/NUM_VEHICLES if USE_DESIRED_AGENTS else PERC_AGENT_CARS*PERC_UNI_CARS
    NUM_AGENTS = NUM_VEHICLES*PERC_AGENT_CARS
    traci.start(['sumo'+('-gui' if SHOW_GUI else ''),'-c','osm_'+str(SCENARIO)+'.sumocfg','--step-length',str(STEP_SIZE)])
    graphdict = mapdata.graphdict
    net = mapdata.net
    targets = mapdata.targets
    edgelist = mapdata.edgelist
    destinations = mapdata.destinations
    NUM_FOES = int(NUM_VEHICLES*PERC_UNI_CARS)-NUM_AGENTS
    vehs = spawnUncontrolledCars(int(NUM_FOES),mapdata) # load uncontrolled cars
    agents,end_edge = spawnControlledCars(NUM_AGENTS,mapdata,NUM_ALGS,vehs,ONLINE) # load controlled cars
    print('loaded vehicles')
    totarrived = 0
    prev_edge = {} # data structure keeping track of previous edges of each vehicle
    next_edge = {} # data structure to allow agents to keep track of their next edge
    # counters for seconds and minutes to allow realistic spawning of buses
    secondtot = 1/STEP_SIZE
    secondcounter = 0
    mcounter = -1
    scounter = -1
    busdata = bus_parser(START_TIME,SCENARIO)
    tottoarrive = int(round(PERC_UNI_CARS*NUM_VEHICLES))
    insim = [] # data structure to keep track of vehicles in the simulation
    print('ready to go')
    correctlyarrived = [] # data structure to keep track of vehicles that disappeared from the simulation when they were supposed to
    works = mapdata.works if CONSIDER_WORKS else {}
    signalled_works = [] # data structure to keep track of signalled wip areas
    for w in works:
        traci.edge.setParameter(w,'color',10)
    # create behaviour data structures
    behaviour_created = []
    behaviour_db = np.zeros((len(targets)*NUM_ALGS, len(edgelist), len(edgelist)))
    # check coloring flags from sim_congif.txt file
    checkfromfile = True
    colorpars = []
    try:
        f = open('sim_config.txt','r')
        colorparams = f.readline()
        f.close()
        colorpars = colorparams.split(';')
    except:
        checkfromfile = False
    colorstreet = False if not checkfromfile else eval(colorpars[2])
    colorreward = False if not checkfromfile else eval(colorpars[4])
    colorpaths = False if not checkfromfile else eval(colorpars[3])
    rewards4colors = {}
    paths_db = {} if ONLINE else load_offline_paths(SCENARIO,NUM_ALGS,CONSIDER_WORKS) # use offline paths if offline, else initialize data structure
    # initializing metrics arrays
    agent_co2s = []
    agent_fuels = []
    agent_noises = []
    agent_cos = []
    agent_pmxs = []
    agent_noxs = []
    agent_hcs = []
    foes_co2s = []
    foes_fuels = []
    foes_noises = []
    foes_cos = []
    foes_pmxs = []
    foes_noxs = []
    foes_hcs = []
    measure = True
    played_audio_edge = None
    while True:
        if len([x for x in insim if not x.__contains__('bus') and not x.__contains__('random')])==0 and totarrived>=1: # if simulation is empty and at least 1 vehicle arrived, stop loop (it works under the hypothesis of the simulation always having a vehicle inside)
            break
        traci.simulationStep() # step of the loop
        vehicles_in_sim = traci.vehicle.getIDList()
        temp_agent_co2s = []
        temp_agent_fuels = []
        temp_agent_noises = []
        temp_agent_cos = []
        temp_agent_pmxs = []
        temp_agent_noxs = []
        temp_agent_hcs = []
        temp_foes_co2s = []
        temp_foes_fuels = []
        temp_foes_noises = []
        temp_foes_cos = []
        temp_foes_pmxs = []
        temp_foes_noxs = []
        temp_foes_hcs = []
        prev_signalled_works = len(signalled_works)
        for vehicle in vehs:
            # look for vehicles in the simulation
            if vehicle in vehicles_in_sim:
                if vehicle not in insim:
                    insim.append(vehicle)
            else:
                if vehicle in insim:
                    insim.remove(vehicle)
                    vehs[vehicle].arrived = True
                    if not vehicle.__contains__('bus') and not vehicle.__contains__('random'):
                        totarrived += 1
            if vehicle in insim: # for every vehicle in the simulation
                # save data for the vehicle
                vehicle_speed = traci.vehicle.getSpeed(vehicle)
                vehicle_fuel = traci.vehicle.getFuelConsumption(vehicle)*STEP_SIZE
                vehicle_noise = traci.vehicle.getNoiseEmission(vehicle)
                vehicle_co2 = traci.vehicle.getCO2Emission(vehicle)*STEP_SIZE
                vehicle_co = traci.vehicle.getCOEmission(vehicle)*STEP_SIZE
                vehicle_pmx = traci.vehicle.getPMxEmission(vehicle)*STEP_SIZE
                vehicle_nox = traci.vehicle.getNOxEmission(vehicle)*STEP_SIZE
                vehicle_hc = traci.vehicle.getHCEmission(vehicle)*STEP_SIZE
                vehicle_waiting = traci.vehicle.getWaitingTime(vehicle)
                if vehicle.__contains__('agent'):
                    temp_agent_co2s.append(vehicle_co2)
                    temp_agent_noises.append(vehicle_noise)
                    temp_agent_fuels.append(vehicle_fuel)
                    temp_agent_cos.append(vehicle_co)
                    temp_agent_pmxs.append(vehicle_pmx)
                    temp_agent_hcs.append(vehicle_hc)
                    temp_agent_noxs.append(vehicle_nox)
                elif vehicle.__contains__('veh'):
                    temp_foes_co2s.append(vehicle_co2)
                    temp_foes_fuels.append(vehicle_fuel)
                    temp_foes_noises.append(vehicle_noise)
                    temp_foes_cos.append(vehicle_co)
                    temp_foes_pmxs.append(vehicle_pmx)
                    temp_foes_hcs.append(vehicle_hc)
                    temp_foes_noxs.append(vehicle_nox)
                vehs[vehicle].speeds.append(vehicle_speed)
                vehs[vehicle].dist = traci.vehicle.getDistance(vehicle)
                vehs[vehicle].fuelconsumption.append(vehicle_fuel)
                if vehs[vehicle].waitingtime[-1]==0:
                    if vehicle_waiting>0:
                        vehs[vehicle].waitingtime[-1] = vehicle_waiting
                else:
                    if vehicle_waiting==0:
                        vehs[vehicle].waitingtime.append(0)
                    else:
                        vehs[vehicle].waitingtime[-1] = vehicle_waiting
                vehs[vehicle].co2emission.append(vehicle_co2)
                vehs[vehicle].coemission.append(vehicle_co)
                vehs[vehicle].noxemission.append(vehicle_nox)
                vehs[vehicle].pmxemission.append(vehicle_pmx)
                vehs[vehicle].hcemission.append(vehicle_hc)
                vehs[vehicle].noiseemission.append(vehicle_noise)
                vehs[vehicle].traveltime += (1 if secondcounter%secondtot==0 else 0) # travel time measure by the second
                roadid = traci.vehicle.getRoadID(vehicle)
                if secondcounter%secondtot == 0 and not roadid.__contains__(':'): # emit random signalling of wip area
                    vehs[vehicle].currentroad = roadid
                    randomSignal(0.005,vehs[vehicle],roadid,works)
                if roadid in works and not roadid.__contains__(':'): # if vehicle is on a wip area
                    if secondcounter%secondtot == 0: # every simulation second
                        if tweet(works,roadid,vehs,end_edge,mapdata,False,vehs[vehicle],LANG,WORKS_THRESHOLD): # try to signal the area
                            if roadid not in signalled_works: # updating signalled_works data structure
                                signalled_works.append(roadid)
                                traci.edge.setParameter(roadid,'color',12)
                    traci.vehicle.setSpeed(vehicle,net.getEdge(roadid).getSpeed()/20) # slowing cars down in wip areas
                elif roadid not in works:
                    traci.vehicle.setSpeed(vehicle,-1)
                # if vehicle is an agent and it enters a new edge/crossing, launch the decision making step
                if vehicle.__contains__('agent') and (vehicle not in prev_edge or vehicle in prev_edge and prev_edge[vehicle]!=roadid) and (vehicle in next_edge and next_edge[vehicle]!=end_edge[vehicle] or vehicle not in next_edge) and len(roadid)>2:
                    time1 = 0 # start counting if time measure
                    if measure and MEASURING:
                        time1 = time.time()
                    currentid = roadid if not roadid.__contains__(':') else next_edge[vehicle]
                    statecars = edgelist.index(net.getEdge(currentid)) # find car state (index of currentid edge)
                    # identify valid neighbours both with and without target edge
                    vn = []
                    vn.append(net.getEdge(currentid))
                    vn.extend(valid_neighbors(net.getEdge(currentid),mapdata,end_edge[vehicle]))
                    vn2 = []
                    vn2.append(net.getEdge(currentid))
                    vn2.extend(valid_neighbors(net.getEdge(currentid),mapdata))
                    prox_edge = None
                    paths = None
                    indmin = -1
                    selpath = None
                    if end_edge[vehicle] in vn: # if destination is among neighbours, move towards it
                        prox_edge = end_edge[vehicle]
                    else: # destination is not among the neighbours, proceed with decision making
                        ss = state_space(statecars,T_HORIZON,mapdata,end_edge[vehicle]) # identify reduced state space
                        ss_edges = conv_ss2edges(ss,edgelist) # turn indexes into edges
                        
                        for s in vn: # look for works among the neighbours
                            if s.getID() in works:
                                if tweet(works,s.getID(),vehs,end_edge,mapdata,True,vehs[vehicle],LANG,WORKS_THRESHOLD): # try to signal the area
                                    if s.getID() not in signalled_works: # updating signalled_works data structure
                                        signalled_works.append(s.getID())
                                        traci.edge.setParameter(s.getID(),'color',12)
                        if ONLINE and ((currentid,end_edge[vehicle]) not in behaviour_created or len(signalled_works)!=prev_signalled_works): # if behaviours are created online and no behaviours are associated to the edge and the target (or update is needed because of works)
                                # create/update pmfs and available paths
                                behaviour_db,paths = online_create_behaviours(mapdata,NUM_ALGS,end_edge[vehicle],ss_edges,behaviour_db,behaviour_created,signalled_works,len(signalled_works)!=prev_signalled_works)
                                for p in paths:
                                    if p not in paths_db:
                                        paths_db[p] = [None]*(len(targets)*NUM_ALGS)
                                    for i in range(NUM_ALGS):
                                        paths_db[p][agents[vehicle].targetIndex+i] = paths[p][i]
                                for v in ss_edges:
                                    if (v.getID(),end_edge[vehicle]) not in behaviour_created or len(signalled_works)!=prev_signalled_works:
                                        behaviour_created.append((v.getID(),end_edge[vehicle]))
                                prev_signalled_works = len(signalled_works)
                              
                        if len(vn)>2: # there is more than 1 valid neighbour
                            r = compute_reward(ss,mapdata,signalled_works,vehs[vehicle],paths_db,agents)
                            if colorreward:
                                colorMap(r,mapdata,rewards4colors,ss_edges)
                                for stre in edgelist:
                                    if not any(stre.getID() in paths_db[currentid][agents[vehicle].targetIndex+checindex] for checindex in range(NUM_ALGS)):
                                        traci.edge.setParameter(stre.getID(),'color',0)
                            new_state,indmin = agents[vehicle].receding_horizon_DM(statecars,T_HORIZON,ss,r,ONLINE,behaviour_db,edgelist) # use crowdsourcing algorithm to select the next state
                            prox_edge = edgelist[new_state] # find newly-selected edge
                            # update VehicleData attributes and variable for selected path
                            if ONLINE:
                                if indmin>=0:
                                    selpath = paths_db[currentid][agents[vehicle].targetIndex+indmin]
                                    vehs[vehicle].selected_id = indmin
                                    if paths_db[currentid][agents[vehicle].targetIndex+indmin][1]!=prox_edge.getID():
                                        print('warning, next edge discrepancy between path and crowdsourcing selection')
                            else:
                                if indmin>=0:
                                    selpath = paths_db[currentid][agents[vehicle].targetIndex+indmin]
                                    vehs[vehicle].selected_id = indmin
                        elif len(vn)==2: # only one road is possible
                            prox_edge = vn[1] # select next edge as destination
                            # update VehicleData attributes and variable for selected path
                            indmin = vehs[vehicle].selected_id
                            selpath = paths_db[currentid][agents[vehicle].targetIndex+indmin]
                        else: # edge is destination
                            prox_edge = vn[0]
                            indmin = vehs[vehicle].selected_id
                            selpath = paths_db[currentid][agents[vehicle].targetIndex+indmin]
                    if len(vn2)>2: # upon crossing with multiple possible selections, play audio
                        if NUM_AGENTS==1 and currentid==roadid and PLAY_AUDIO:
                            playAudio(mapdata,currentid,prox_edge,LANG,roundabout=(currentid in mapdata.streets_in_roundabouts))
                    else: # look for next crossing, then play audio
                        if NUM_AGENTS==1 and PLAY_AUDIO and selpath is not None:
                            true_play_audio, in_roundabout, dist, audio_edge, prev_audio_edge = search_next(mapdata,paths_db[currentid][agents[vehicle].targetIndex+indmin])
                            if currentid in mapdata.streets_in_roundabouts: # next selection is inside a roundabout, no need for audio play
                                true_play_audio = False
                            if true_play_audio and audio_edge!=played_audio_edge:
                                playAudio(mapdata,currentid,net.getEdge(audio_edge),LANG,dist,net.getEdge(prev_audio_edge),in_roundabout,path=(None if not in_roundabout or paths_db[currentid] is None else paths_db[currentid][agents[vehicle].targetIndex+indmin]))
                                played_audio_edge = audio_edge
                    next_edge[vehicle] = prox_edge.getID()
                    if prox_edge.getID() == end_edge[vehicle]:
                        print(vehicle+' ARRIVED')
                        if vehicle not in correctlyarrived:
                            correctlyarrived.append(vehicle)
                    print('time ['+str(int(secondcounter/secondtot))+':'+str(int(secondcounter%secondtot))+'] - '+str(vehicle)+' on edge '+str(currentid)+' target '+str(prox_edge.getID())+' current target '+str(traci.vehicle.getRoute(vehicle)[-1]))
                    # update vehicle SUMO route according to the selection by the crowdsourcing algorithm
                    traci.vehicle.setRoute(vehicle,traci.simulation.findRoute(roadid,prox_edge.getID()).edges)
                    print('selected edge '+str(prox_edge.getID())+' selected index '+str(vehs[vehicle].selected_id))
                    if measure and MEASURING: # if measuring, stop counter and save measures
                        time2 = time.time()
                        f = open('time_measures.txt','a')
                        f.write(str(ONLINE)+':'+str(T_HORIZON)+':'+str(len(ss_edges))+':'+str(time2-time1)+'\n')
                        f.close()
                    if colorstreet and ((colorpaths) or (selpath is not None and not colorpaths)):
                        for stre in edgelist:
                            if stre.getID() not in works:
                                traci.edge.setParameter(stre.getID(),'color',0)
                        if colorpaths: # color paths of different behaviours
                            for selpat in range(NUM_ALGS):
                                colorv = getIfromRGB(list(alg_color(index2alg(selpat)))[0:3])
                                for e in paths_db[currentid][agents[vehicle].targetIndex+selpat]:
                                    if e not in works:
                                        traci.edge.setParameter(e,'color',colorv)
                        else: # color selected path only
                            colorv = getIfromRGB(list(alg_color(index2alg(indmin)))[0:3])
                            for e in selpath:
                                if e not in works:
                                    traci.edge.setParameter(e,'color',colorv)
                        if vehicle in prev_edge:
                            traci.edge.setParameter(prev_edge[vehicle],'color',0)
                print('time ['+str(int(secondcounter/secondtot))+':'+str(int(secondcounter%secondtot))+'] - '+str(vehicle)+' on edge '+str(roadid))
                prev_edge[vehicle] = roadid
        if secondcounter%secondtot == 0: # if second has passed, check for spawn of bus or random car
            scounter += 1
            if scounter%60 == 0:
                mcounter += 1
                if INCLUDE_RANDOM and tottoarrive+1<NUM_VEHICLES: # if possible, spawn random vehicle
                    spawned, spawnedRand = spawnRandom(graphdict)
                    if spawned:
                        print("Spawned random vehicle")
                        # tottoarrive += 1 if NUM_AGENTS==0 else 0
                        vehs[spawnedRand] = VehicleData(spawnedRand,spawnedRand.replace('_vehicle',''),scounter*(mcounter+1),'')
                if INCLUDE_BUS and tottoarrive+1<NUM_VEHICLES: # if possible, spawn bus
                    spawned, spawnedBuss = spawnBus(busdata,mcounter)
                    if spawned:
                        print("Spawned bus")
                        # tottoarrive += len(spawnedBuss) if NUM_AGENTS==0 else 0
                        for el in spawnedBuss:
                            vehs[el] = VehicleData(el,el+'route',scounter*(mcounter+1),'')
        secondcounter += 1
        # update data structures keeping track of metrics
        agent_co2s.append(0 if len(temp_agent_co2s)==0 else np.mean(temp_agent_co2s))
        agent_noises.append(0 if len(temp_agent_noises)==0 else np.mean(temp_agent_noises))
        agent_fuels.append(0 if len(temp_agent_fuels)==0 else np.mean(temp_agent_fuels))
        agent_cos.append(0 if len(temp_agent_cos)==0 else np.mean(temp_agent_cos))
        agent_pmxs.append(0 if len(temp_agent_pmxs)==0 else np.mean(temp_agent_pmxs))
        agent_noxs.append(0 if len(temp_agent_noxs)==0 else np.mean(temp_agent_noxs))
        agent_hcs.append(0 if len(temp_agent_hcs)==0 else np.mean(temp_agent_hcs))
        foes_co2s.append(0 if len(temp_foes_co2s)==0 else np.mean(temp_foes_co2s))
        foes_noises.append(0 if len(temp_foes_noises)==0 else np.mean(temp_foes_noises))
        foes_fuels.append(0 if len(temp_foes_fuels)==0 else np.mean(temp_foes_fuels))
        foes_cos.append(0 if len(temp_foes_cos)==0 else np.mean(temp_foes_cos))
        foes_pmxs.append(0 if len(temp_foes_pmxs)==0 else np.mean(temp_foes_pmxs))
        foes_noxs.append(0 if len(temp_foes_noxs)==0 else np.mean(temp_foes_noxs))
        foes_hcs.append(0 if len(temp_foes_hcs)==0 else np.mean(temp_foes_hcs))
        
    # loop is broken, time to return values
    retds = []
    # gathering data of all vehicles that correctly arrived and that traversed paths of interest
    for vehicle in vehs:
        if vehs[vehicle].arrived and (not vehicle.__contains__('bus') and not vehicle.__contains__('random')):
            retds.append((vehicle,
                          vehs[vehicle].alg,
                          (0 if len(vehs[vehicle].speeds)==0 else np.mean(vehs[vehicle].speeds)),
                          vehs[vehicle].dist,
                          sum(vehs[vehicle].fuelconsumption),
                          sum(vehs[vehicle].waitingtime),
                          sum(vehs[vehicle].noiseemission),
                          sum(vehs[vehicle].co2emission),
                          vehs[vehicle].traveltime,
                          sum(vehs[vehicle].coemission),
                          sum(vehs[vehicle].pmxemission),
                          sum(vehs[vehicle].hcemission),
                          sum(vehs[vehicle].noxemission)
                        ))
    traci.close()
    
    return retds,NUM_AGENTS,correctlyarrived
