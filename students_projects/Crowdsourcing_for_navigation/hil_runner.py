# Module for running vehicle-in-the-loop simulations

from threading import Thread
from pynmeagps import NMEAReader
import serial
from time import sleep

from algorithms import connection_exists
from vehicledata import VehicleData
import random
import traci
from agent import Agent
import numpy as np
from bus_parser import bus_parser
from behaviours_maker import valid_neighbors
from behaviours_maker import online_create_behaviours
from intercar_comm import tweet
from spawners import *
from colors import *
from text2speech_handler import playAudio
from traci import TraCIException
from single_simulation import compute_reward
from single_simulation import randomSignal
from single_simulation import state_space
from single_simulation import conv_ss2edges
from single_simulation import load_offline_paths
from text2speech_handler import search_next

# class to emulate results of NMEAReader class, useful for testing with pre-recorded GPS locations
class Message():
    def __init__(self,lat,lon):
        self.lat = lat # latitude
        self.lon = lon # longitude

# thread for reading GPS locations from serial port, inputs are: 
# - serial port
def reading_thread(ser):
    verbose = False
    global msg
    global stop
    stop = False
    print('thread is on')
    msg = None
    try:
        nmr = NMEAReader(ser)
        while True:
            (raw_data, msg2) = nmr.read() # msg will be global variable that main will read
            msgstr = str(msg2)
            if msgstr.__contains__('lat=') and msgstr.__contains__('lon=') and (not msgstr.__contains__('lat=,') and not msgstr.__contains__('lon=,')): # if latitude and longitude are included in the message
                f = open('log_gps_test.txt','a')
                msg = msg2 # update msg variable
                if verbose:
                    print(f"lat: {msg.lat} - lon: {msg.lon}")
                f.write(str(msg.lat)+':'+str(msg.lon)+'\n') # save msg content on file for future testing using pre-recorded locations
                f.close()
    except KeyboardInterrupt:
        ser.close()

# thread for reading pre-recorded GPS locations from file, inputs are:
# - data structure containing all static data related to the map
def reading_thread_file(mapdata):
    global msg
    global stop
    print('thread is on')
    msg = None
    stop = False
    coords = []
    f = open('log_gps_test.txt','r')
    dats = f.readlines()
    for dat in dats:
        cs = dat.split(':')
        coords.append([float(cs[0]),float(cs[1])])
    i = 0
    sleep(3.0)
    for i in range(len(coords)):
        msg = Message(coords[i][0],coords[i][1]) # update msg according to latitude and longitude coming from file
        print(coords[i])
        while not proceed: # coordinating with simulation steps
            pass
        sleep(0.13)
    stop = True
    exit()

# method for sorting, inputs are:
# - element to sort
def take_first(elem):
    return elem[0]

# method for finding closest edge to the coordinates given by msg, inputs are:
# - data structure containing all static data related to the map,
# - pre-determined vehicle route (if None, it allows to look for any traversable edge in the map)(optional),
# - edge from which the vehicle needs to move (if None, it allows to look for any traversable edge in the path, if path is not None)(optional)
def find_closest_edge(mapdata,path=None,start_edge=None):
    verbose = True
    if verbose:
        print('FINDING CLOSEST EDGE')
    net = mapdata.net
    radius = 20
    x, y = net.convertLonLat2XY(msg.lon, msg.lat) 
    ne_edges = net.getNeighboringEdges(x, y, radius,includeJunctions=False) # get all edges close to the coordinates
    distancesAndEdges_full = sorted([(dist, edge) for edge, dist in ne_edges], key=take_first) # sort them by distance
    distancesAndEdges = []
    if verbose:
        print(path)
        print(start_edge)
    for dae in distancesAndEdges_full: # filter edges
        if verbose:
            print(dae[1])
        if dae[1].allows('passenger'): # take into account only traversable edges
            if path is None: # edges can belong to no path to be taken into account
                distancesAndEdges.append(dae)
            else: # only edges in the path can be taken into account
                if dae[1].getID() in path: # the edge is in the path
                    if start_edge is not None: # if the vehicle needs to move from a specific edge
                        p2 = traci.simulation.findRoute(start_edge,dae[1].getID()).edges
                        if len(p2)>0 and len(p2)<4: # if the edge is not too far from the specific edge, include it
                            distancesAndEdges.append(dae)
                            print(p2)
                    else: # include in the path
                        distancesAndEdges.append(dae)
                else: # the edge is not in the path
                    if start_edge is not None: # if the vehicle needs to move from a specific edge
                        p2 = traci.simulation.findRoute(start_edge,dae[1].getID()).edges
                        # if len(p2)>0 and len(p2)<4: # if the edge is not too far from the specific edge, include it even if it does not belong to the path
                        #     distancesAndEdges.append(dae)
                        #     print(p2)
    if verbose:
        print(distancesAndEdges)
    if len(distancesAndEdges)==0: # no suitable edge found
        return None,0,0
    dist, closestEdge = distancesAndEdges[0]
    initial_edge = closestEdge.getID()
    if verbose:
        print('x: '+str(x)+' y: '+str(y)+' '+str(net.getEdge(initial_edge)))
    return initial_edge,x,y

# method for checking if audio must be played by reading parameters on file
def check_audio_from_params():
    f = open('sim_config.txt','r')
    c = f.readline().split(';')[5]
    c = eval(c)
    f.close()
    return c

WORKS_THRESHOLD = 1 # threshold value after which an area is considered to be "work-in-progress"

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
    LANG = 'en'
    SCENARIO = mapdata.scenario
    PERC_AGENT_CARS = DESIRED_AGENTS/NUM_VEHICLES if USE_DESIRED_AGENTS else PERC_AGENT_CARS*PERC_UNI_CARS
    NUM_AGENTS = NUM_VEHICLES*PERC_AGENT_CARS
    PLAY_AUDIO = check_audio_from_params()
    verbose = False # many prints ahead, change to True for debugging purposes
    open('log_gps_test.txt','w').close() # decomment if using real GPS
    ser = serial.Serial('/dev/rfcomm0', 9600) # decomment if using real GPS
    global proceed
    proceed = False
    # create a thread
    thread = Thread(target=reading_thread, args=[ser]) # decomment if using real GPS
    # thread = Thread(target=reading_thread_file, args=[mapdata]) # decomment if using pre-recorded GPS locations
    # run the thread
    thread.daemon = True
    thread.start()
    traci.start(['sumo'+('-gui' if SHOW_GUI else ''),'-c','osm_'+str(SCENARIO)+'.sumocfg','--step-length',str(STEP_SIZE)])
    graphdict = mapdata.graphdict
    net = mapdata.net
    targets = mapdata.targets
    edgelist = mapdata.edgelist
    destinations = mapdata.destinations
    while msg is None: # GPS signal has yet to be received
        pass
    print('message read')
    print('looking for initial edge')
    initial_edge,x,y = find_closest_edge(mapdata)
    while initial_edge is None: # looking for an initial edge as long as no one is found
        initial_edge, x, y = find_closest_edge(mapdata)
        proceed = True
    proceed = False
    out_of_course_counter = 0
    if initial_edge is not None:
        initial_edge,x,y = find_closest_edge(mapdata)
        vehs = spawnUncontrolledCars(int((PERC_UNI_CARS-PERC_AGENT_CARS)*NUM_VEHICLES),mapdata)
        agents,end_edge = spawnControlledCars(NUM_AGENTS,mapdata,NUM_ALGS,vehs,ONLINE,initial_edge)
        print('loaded vehicles')
        agent_car = list(agents.keys())[0] # first vehicle among the agents is the one controlled by GPS
        traci.vehicle.moveToXY(agent_car,initial_edge,0,x,y)
        totarrived = 0
        prev_edge = {} # data structure keeping track of previous edges of each vehicle
        cl_prev_edge = {} # data structure keeping track of previous edges of each vehicle, excluding crossings
        next_edge = {} # data structure to allow agents to keep track of their next edge
        secondtot = 1/STEP_SIZE
        # counters for seconds and minutes to allow realistic spawning of buses
        secondcounter = 0
        mcounter = -1
        scounter = -1
        busdata = bus_parser(START_TIME,SCENARIO)
        tottoarrive = PERC_UNI_CARS*NUM_VEHICLES
        insim = [] # data structure to keep track of vehicles in the simulation
        print('ready to go')
        correctlyarrived = [] # data structure to keep track of vehicles that disappeared from the simulation when they were supposed to
        works = mapdata.works if CONSIDER_WORKS else {}
        signalled_works = [] # data structure to keep track of signalled wip areas
        for w in works:
            traci.edge.setParameter(w,'color',10)
        passed = {} # data structure to keep track of which vehicles went where (THIS WILL PROBABLY BE REMOVED AFTER UPDATES TO THE COST FUNCTION)
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
        # additional variables for audio and car placing in the map
        current_edge = None
        played_audio_edge = None
        indmin = -1
        while True:
            proceed = False
            move = False
            if len([x for x in insim if not x.__contains__('bus') and not x.__contains__('random')])==0 and totarrived>=1: # if simulation is empty and at least 1 vehicle arrived, stop loop (it works under the hypothesis of the simulation always having a vehicle inside)
                break
            traci.simulationStep() # step of the loop
            vehicles_in_sim = traci.vehicle.getIDList()
            proceed = True
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
                    roadid = traci.vehicle.getRoadID(vehicle) # check on which road the vehicle is
                    try:
                        if vehicle.__contains__('agent') and not stop: # if the vehicle is the agent, move it according to GPS signal
                            old_x = x
                            old_y = y
                            checkr = None
                            if agent_car in cl_prev_edge:
                                if not roadid.__contains__(':'):
                                    checkr = roadid
                                else:
                                    checkr = cl_prev_edge[agent_car]
                            else:
                                checkr = roadid
                            kR = 1 # move vehicle by keeping its route
                            current_edge,x,y = find_closest_edge(mapdata,path=(None if indmin<0 else paths_db[checkr][agents[vehicle].targetIndex+indmin]),start_edge=(None if indmin<0 or agent_car not in cl_prev_edge else cl_prev_edge[agent_car])) # look for next edge in the path
                            if current_edge is None: # no edge found, try with no limitations introduced by the path
                                current_edge,x,y = find_closest_edge(mapdata)
                                kR = 0 # move vehicle without keeping the route
                            out_of_course_counter,move = move_car_on_road(mapdata,roadid,agent_car,cl_prev_edge,current_edge,None if indmin<0 else paths_db[checkr][agents[vehicle].targetIndex+indmin],x,y,old_x,old_y,out_of_course_counter,kR=kR) # move car
                    except TraCIException as e: # move failed for any reason (usually no suitable edge was found), move the car as if it were in a simulated environment
                        move = False
                        traci.vehicle.setSpeed(agent_car,-1)
                        print('exception here, '+str(e))
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
                    if vehicle not in passed:
                        passed[vehicle] = {}
                    if verbose:
                        print('roadid '+str(roadid))
                    if not roadid.__contains__(':'): # update cl_prev_edge
                        if vehicle not in cl_prev_edge or roadid!=cl_prev_edge[vehicle]:
                            if vehicle.__contains__('agent'):
                                if move:
                                    cl_prev_edge[vehicle] = current_edge
                                else:
                                    cl_prev_edge[vehicle] = roadid
                            else:
                                cl_prev_edge[vehicle] = roadid
                    if verbose:
                        print('cl prev edge '+str(cl_prev_edge[vehicle]))
                    if agent_car==vehicle and current_edge is not None and move and not roadid.__contains__(':'):
                        roadid = cl_prev_edge[agent_car]
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
                    elif roadid not in works and vehicle!=agent_car:
                        traci.vehicle.setSpeed(vehicle,-1)
                    if vehicle == agent_car and not thread.is_alive:
                        print('thread is not alive')
                        traci.vehicle.setSpeed(agent_car,-1)
                    # if vehicle is an agent and it enters a new edge/crossing, launch the decision making step
                    if vehicle.__contains__('agent') and (vehicle not in prev_edge or vehicle in prev_edge and prev_edge[vehicle]!=roadid) and (vehicle in next_edge and next_edge[vehicle]!=end_edge[vehicle] or vehicle not in next_edge) and len(roadid)>2:
                        # currentid = roadid if not roadid.__contains__(':') else next_edge[vehicle] # identify id of edge on which make the evaluations
                        currentid = current_edge # identify id of edge on which make the evaluations
                        if verbose:
                            print('currentid '+str(currentid))
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
                            
                            for s in ss_edges: # look for works among the neighbours
                                if s.getID() in works:
                                    if tweet(works,s.getID(),vehs,end_edge,mapdata,True,vehs[vehicle],LANG,WORKS_THRESHOLD): # try to signal the area
                                        if s.getID() not in signalled_works: # updating signalled_works data structure
                                            signalled_works.append(s.getID())
                                            traci.edge.setParameter(s.getID(),'color',12)
                            if ONLINE and ((currentid,end_edge[vehicle]) not in behaviour_created or len(signalled_works)!=prev_signalled_works): # if behaviours are created online and no behaviours are associated to the edge and the target (or update is needed because of works)
                                    print('producing behaviours ')
                                    # create/update pmfs and available paths
                                    behaviour_db,paths = online_create_behaviours(mapdata,NUM_ALGS,end_edge[vehicle],ss_edges,behaviour_db,behaviour_created,signalled_works,len(signalled_works)!=prev_signalled_works)
                                    for p in paths:
                                        if p not in paths_db:
                                            paths_db[p] = [None]*len(targets)
                                        paths_db[p][agents[vehicle].targetIndex] = paths[p]
                                    for v in ss_edges:
                                        if (v.getID(),end_edge[vehicle]) not in behaviour_created or len(signalled_works)!=prev_signalled_works:
                                            behaviour_created.append((v.getID(),end_edge[vehicle]))
                                
                            if len(vn)>2 or len(signalled_works)!=prev_signalled_works: # there is more than 1 valid neighbour
                                r = compute_reward(ss,mapdata,signalled_works,vehs[vehicle],paths_db,agents) # compute reward
                                if colorreward:
                                    colorMap(r,mapdata,rewards4colors,ss_edges)
                                new_state,indmin = agents[vehicle].receding_horizon_DM(statecars,T_HORIZON,ss,r,ONLINE,behaviour_db,edgelist) # use crowdsourcing algorithm to select the next state
                                prox_edge = edgelist[new_state] # find newly-selected edge
                                # update VehicleData attributes and variable for selected path
                                if ONLINE:
                                    if paths is not None and indmin>=0:
                                        selpath = paths[currentid][indmin]
                                        vehs[vehicle].selected_id = indmin
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
                        if PLAY_AUDIO:
                            if len(vn2)>2: # upon crossing with multiple possible selections, play audio
                                if NUM_AGENTS==1 and currentid==roadid:
                                    playAudio(mapdata,currentid,prox_edge,LANG,roundabout=(currentid in mapdata.streets_in_roundabouts))
                            else: # look for next crossing, then play audio
                                if NUM_AGENTS==1 and selpath is not None:
                                    true_play_audio, in_roundabout, dist, audio_edge, prev_audio_edge = search_next(mapdata,paths_db[currentid][agents[vehicle].targetIndex+indmin])
                                    if currentid in mapdata.streets_in_roundabouts: # next selection is inside a roundabout, no need for audio play
                                        true_play_audio = False
                                    if true_play_audio and audio_edge!=played_audio_edge:
                                        playAudio(mapdata,currentid,net.getEdge(audio_edge),LANG,dist,net.getEdge(prev_audio_edge),in_roundabout,path=(None if not in_roundabout or paths_db[currentid] is None else paths_db[currentid][agents[vehicle].targetIndex+indmin]))
                                        played_audio_edge = audio_edge
                            if len(signalled_works)!=prev_signalled_works:
                                played_audio_edge = None
                        prev_signalled_works = len(signalled_works)
                        next_edge[vehicle] = prox_edge.getID()
                        if prox_edge.getID() == end_edge[vehicle]:
                            print(vehicle+' ARRIVED')
                            if vehicle not in correctlyarrived:
                                correctlyarrived.append(vehicle)
                        # update vehicle SUMO route according to the selection by the crowdsourcing algorithm
                        rt = paths_db[currentid][agents[vehicle].targetIndex+indmin]
                        if rt is not None:
                            rt2 = []
                            if verbose:
                                print('changing path')
                            rt2.extend(rt[0:4])
                            try:
                                traci.vehicle.setRoute(agent_car,rt2)
                            except TraCIException:
                                pass
                        if colorstreet and ((colorpaths) or (selpath is not None and not colorpaths)): # if flags are okay, color selected street or different possible paths
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
                        print('selected edge '+str(prox_edge.getID())+' selected index '+str(vehs[vehicle].selected_id))
                    
                    prev_edge[vehicle] = roadid
                    if roadid not in passed[vehicle]:
                        passed[vehicle][roadid] = 1
                    else:
                        passed[vehicle][roadid] += 1
                        
            if secondcounter%secondtot == 0: # if second has passed, check for spawn of bus or random car
                scounter += 1
                if scounter%60 == 0:
                    mcounter += 1
                    if INCLUDE_RANDOM: # if possible, spawn random vehicle
                        spawned, spawnedRand = spawnRandom(graphdict)
                        if spawned:
                            if verbose:
                                print("Spawned random vehicle")
                            vehs[spawnedRand] = VehicleData(spawnedRand,spawnedRand.replace('_vehicle',''),scounter*(mcounter+1),'')
                    if INCLUDE_BUS: # if possible, spawn bus
                        spawned, spawnedBuss = spawnBus(busdata,mcounter)
                        if spawned:
                            if verbose:
                                print("Spawned bus")
                            for el in spawnedBuss:
                                vehs[el] = VehicleData(el,el+'route',scounter*(mcounter+1),'')
            secondcounter += 1 # update time counter
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
        proceed = True
        retds = []
        # gathering data of all vehicles that correctly arrived and that traversed paths of interest
        for vehicle in vehs:
            if vehs[vehicle].arrived and (not vehicle.__contains__('bus') or not vehicle.__contains__('random')):
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
    else:
        return None

# method for establishing if car has to be moved by GPS or by TraCI, inputs are:
# - data structure containing all static data related to the map,
# - id of the road on which the car is,
# - id of the agent vehicle,
# - data structure to keep track of previous non-crossing edge,
# - edge identified through the GPS location process,
# - currently selected path,
# - current coordinates obtained from the GPS,
# - previous coordinates obtained from the GPS,
# - counter for out of course edges (UPON ADDITIONAL TESTING, THIS WILL BE REMOVED IF DEEMED UNNECESSARY),
# - value for making car move along its route or not (optional)
def move_car_on_road(mapdata,roadid,agent_car,cl_prev_edge,current_edge,path,x,y,old_x,old_y,out_of_course_counter,kR=1):
    verbose = False # many prints ahead, change to True for debugging
    if verbose:
        print('roadid '+str(roadid)+' current_edge '+str(current_edge))
        print('current route '+str(list(traci.vehicle.getRoute(agent_car))))
    tolerance = 15 # tolerance for out of course counter
    r_prev_edge = agent_car in cl_prev_edge
    a_prev_edge = None if not r_prev_edge else cl_prev_edge[agent_car]
    move = False
    crossing = roadid.__contains__(':')
    if not r_prev_edge: # car just spawned
        move = True
        out_of_course_counter = 0
    else: # car spawned before
        if x == old_x and y == old_y: # no movement detected
            # traci.vehicle.setSpeed(agent_car,0)
            if r_prev_edge:
                if current_edge!=a_prev_edge and not crossing: # different edge detected, wrong detection
                    out_of_course_counter = 1
                else:
                    out_of_course_counter = 0
                    move = True
        else: # movement detected
            traci.vehicle.setSpeed(agent_car,-1)
            if crossing: # if in crossing, assume to move correctly no matter the route
                move = True
            else: # not in crossing
                if current_edge==roadid: # edge is the same, keep moving
                    move = True
                else: # different edges detected, car either moved to the next edge or it is moving to a wrong edge
                    if path is not None: # path to add suggestion
                        if current_edge in path: # detected edge is in the path
                            if len(traci.simulation.findRoute(a_prev_edge,current_edge).edges)<4: # it is pretty close, move there
                                move = True
                                out_of_course_counter = 0
                            else: # take it as a wrong edge
                                if out_of_course_counter<tolerance: # not enough detections to consider it true
                                    move = False
                                    out_of_course_counter += 1
                                else: # many detections in a row occurred, consider it the true path
                                    move = True
                                    out_of_course_counter = 0
                        else:
                            if out_of_course_counter<tolerance: # not enough detections to consider it true
                                move = False
                                out_of_course_counter += 1
                            else: # many detections in a row occurred, consider it the true path
                                move = True
                                out_of_course_counter = 0
                    else: # no path, no worries, move as you like
                        move = True
                        out_of_course_counter = 0
    if verbose:
        print('move: '+str(move))
        print('oocc: '+str(out_of_course_counter))
        print('current_edge: '+str(current_edge))
        print('x,y:         '+str(x)+','+str(y))
        print('old_x,old_y: '+str(old_x)+','+str(old_y))
    if move:
        print('trying to move')
        traci.vehicle.moveToXY(agent_car,current_edge if not crossing else roadid,-1,x,y,keepRoute=kR)
        print('MOVING HERE')
    return out_of_course_counter,move
