# Module for spawning vehicles according to parameters

import traci
import random
from vehicledata import *
from algorithms import build_path
from agent import Agent
from online_crowdsourcing import *
from crowdsourcing import Crowdsourcing

# method for spawning cars with random routes, inputs are:
# - data structure containing data about the road network graph
def spawnRandom(graphdict):
    sourcenode = random.choice(list(graphdict.keys()))
    if graphdict[sourcenode]=={}:
        return False,None
    sourcenoded = random.choice(list(graphdict[sourcenode].keys()))
    source = graphdict[sourcenode][sourcenoded][0]
    sourcenode = random.choice(list(graphdict.keys()))
    if graphdict[sourcenode]=={}:
        return False,None
    sourcenoded = random.choice(list(graphdict[sourcenode].keys()))
    dest = graphdict[sourcenode][sourcenoded][0]
    route = list(traci.simulation.findRoute(source,dest).edges) # look for route between 2 randomly selected nodes
    if len(route)>3: # if route is long enough, spawn vehicle
        rid = 'random_'+str(source)+'_'+str(dest)
        traci.route.add(rid,route)
        traci.vehicle.add(rid+'_vehicle',rid,'Car_RANDOM')
        traci.vehicle.setSpeed(rid+'_vehicle',-1)
        return True,rid+'_vehicle'
    return False,None

# method for spawning buses according to their scheduling, inputs are:
# - data structure containing infos about the buses and their scheduling,
# - simulation time expressed as counter
def spawnBus(busdata,currenttime):
    spawned = False
    spawnedBuss = []
    for tup in busdata:
        if tup[0]==currenttime:
            routedata = busdata[tup]
            vbusid = 'bus_'+str(tup[1]).replace(' ','_')+'_'+str(tup[0])
            if routedata[0][0] is not None and routedata[1][0]:
                if len(routedata)==2:
                    r = traci.simulation.findRoute(routedata[0][0],routedata[1][0]).edges
                    traci.route.add(vbusid+'route',r)
                    traci.vehicle.add(vbusid,vbusid+'route','bus')
                else:
                    r1 = list(traci.simulation.findRoute(routedata[0][0],routedata[1][0]).edges)
                    r2 = list(traci.simulation.findRoute(routedata[1][0],routedata[2][0]).edges)
                    r1.__delitem__(-1)
                    r1.extend(r2)
                    traci.route.add(vbusid+'route',r1)
                    traci.vehicle.add(vbusid,vbusid+'route','bus')
                    traci.vehicle.setSpeed(vbusid,-1)
                spawned = True
                spawnedBuss.append(vbusid)
    return spawned,spawnedBuss

# method for spawning uncontrolled cars in paths of interest, inputs are:
# - number of uncontrolled cars to spawn,
# - data structure containing all static data related to the map
def spawnUncontrolledCars(num_uncontrolled,mapdata):
    destinations = mapdata.destinations
    sources = mapdata.sources
    target_weights = mapdata.target_weights
    vehs = {}
    i = 0
    j = 0
    for c in range(int(num_uncontrolled)):
        # source = random.choices(sources)[0] # choose a random source
        # source = '766350967' # starting edge in Salerno city centre map
        # source = '330223560#1'
        source = sources[c%20]
        # dest = random.choices(destinations,target_weights)[0] # destination selected according to realistic traffic flows
        dest = destinations[c%(len(destinations))] # destination selected in order
        # dest = destinations[0] # selectable destination index
        id = 'veh'+str(c+1)+'_'+str(dest[1])
        routeid = 'route'+str(c+1)
        route = traci.simulation.findRoute(source,dest[0]).edges
        vehs[id] = VehicleData(id,'route'+str(c+1),i,dest[1],route)
        traci.route.add(routeid,route)
        traci.vehicle.add(id,routeid,'Car_AGENT',str(i)) # new uncontrolled car will spawn every 5 seconds
        traci.vehicle.setSpeed(id,-1)
        print(str(id)+' loaded with start '+str(source))
        i += 5
        j += 1
    return vehs

# method for spawning controlled cars in paths of interest, inputs are:
# - number of agents (controlled cars to spawn),
# - data structure containing all static data related to the map,
# - number of behaviours for the crowdsourcing algorithm,
# - data structure containing all VehicleData instances,
# - a flag defining whether behaviours need to be created online or offline,
# - edge from which the controlled cars will start (used for vehicle-in-the-loop simulation)(optional)
def spawnControlledCars(NUM_AGENTS,mapdata,NUM_ALGS,vehs,online,agent_start=None):
    destinations = mapdata.destinations
    targets = mapdata.targets
    sources = mapdata.sources
    target_weights = mapdata.target_weights
    scenario = mapdata.scenario
    start_edge = ('-579690548#1' if scenario=='Unisa' else '766350967') if agent_start is None else agent_start
    begin_spawn = len(vehs)
    end_edge = {}
    agents = {}
    for i in range(int(NUM_AGENTS)):
        start_edge = sources[(begin_spawn+i)%20]
        # destt = random.choices(destinations,target_weights)[0] # destination selected according to realistic traffic flows
        destt = destinations[i%(len(destinations))] # destination selected in order
        # destt = destinations[0] # selectable destination index
        dest = destt[0]
        agentid = 'agent'+str(i)+'_'+str(destt[1])
        agrouteid = agentid+'_route'
        end_edge[agentid] = dest
        agent = Agent(start_edge,list(targets.values()).index(dest)*NUM_ALGS)
        crowds_controller = OnlineCrowdsourcing(agent,NUM_ALGS,mapdata) if online else Crowdsourcing(agent,NUM_ALGS) # if online behaviours must be used, use online crowdsourcing
        agents[agentid] = crowds_controller
        vehs[agentid] = VehicleData(agentid,agrouteid,i*10,destt[1])
        traci.route.add(agrouteid,(start_edge,start_edge))
        traci.vehicle.add(agentid,agrouteid,'Car_'+str(destt[1]),str(i*10)) # a new agent is spawned every 10 seconds
        traci.vehicle.setSpeed(agentid,-1)
        print(agentid+" loaded with start "+str(start_edge))
    return agents,end_edge