# Module for gathering all scripts related to the creation and update of behaviours and paths

import numpy as np
import traci
import os
import sys
import sumolib
from algorithms import build_path
from colors import *
import math

# method for calculating length of a path in terms of travel time, inputs are: 
# - path whose length must be evaluated, 
# - list of work-in-progress areas, 
# - data structure containing all static data related to the map
def calculate_pathlen(path,works,mapdata):
    if path is None:
        return math.inf
    worksweight = 20
    net = mapdata.net
    pathlen = 0
    for j in path:
        if j!='SUCC': # length of path must be calculated only for edges, when achieving success path planning algorithms also add 'SUCC'
            # pathlen += traci.edge.getTraveltime(j)*(1 if j not in works else 10) # use TraCI travel times (same as static travel times)
            pathlen += net.getEdge(j).getLength()/net.getEdge(j).getSpeed()*(1 if j not in works else worksweight) # use static travel times
    return pathlen

# method for calculating length of a path in terms of meters, inputs are: 
# - path whose length must be evaluated, 
# - data structure containing all static data related to the map
def calculate_pathlen_meters(path,mapdata):
    net = mapdata.net
    pathlen = 0
    for j in path:
        if j!='SUCC': # length of path must be calculated only for edges, when achieving success path planning algorithms also add 'SUCC'
            pathlen += net.getEdge(j).getLength() # use static length
    return pathlen

# method for obtaining list of valid neighbours of an edge, inputs are:
# - edge of which neighbours must be found,
# - data structure containing all static data related to the map,
# - target edge of the vehicle looking for valid neighbours (if None, all neighbours of the edge are taken into account)(optional),
# - edgesto ignore while looking for neighbours (optional),
# - flag for taking into account neighbours on the opposite driving direction too (if False, only reachable neighbours are taken into account)(optional)
def valid_neighbors(edge,mapdata,target=None,forbid=None,checkIngoing=False):
    edgegraph = mapdata.edgegraph
    res = []
    n1 = edge.getToNode()
    for e in n1.getOutgoing():
        if e.getID()!=edge.getID() and e.allows('passenger') and e.getID() in edgegraph[edge.getID()]:
            if target is None: # every edge is considered valid
                res.append(e)
            else:
                if not mapdata.edge2target[e.getID()][target][0]: # update data structure keeping track of which edge can reach which target for efficiency
                    mapdata.edge2target[e.getID()][target] = (True,False,[])
                    path = traci.simulation.findRoute(e.getID(),target).edges
                    if len(path)>0: # target is reachable by the edge
                        mapdata.edge2target[e.getID()][target] = (True,True,list(path)) # update data structure to signal reachability
                        if forbid is None: # every edge reaching the target is considered valid
                            res.append(e)
                        else:
                            if forbid not in path: # include edge only if the path is not taking into account a forbidden edge
                                res.append(e)
                    else: # target is unreachable, update data structure to signal unreachability
                        mapdata.edge2target[e.getID()][target] = (True,False,[])
                else: # data structure is already updated
                    if mapdata.edge2target[e.getID()][target][1]: # if edge can reach the target
                        if forbid is None: # no edge is forbidden, include this neighbour
                            res.append(e)
                        else:
                            if forbid not in mapdata.edge2target[e.getID()][target][2]: # include edge only if the path is not taking into account a forbidden edge
                                res.append(e)
    if checkIngoing: # take into account opposite direction edges too
        for e in n1.getIncoming():
            if e.getID()!=edge.getID() and e.allows('passenger') and e.getID() in edgegraph[edge.getID()]:
                if target is None: # every edge is considered valid
                    res.append(e)
                else:
                    if not mapdata.edge2target[e.getID()][target][0]: # update data structure keeping track of which edge can reach which target for efficiency
                        mapdata.edge2target[e.getID()][target] = (True,False,[])
                        path = traci.simulation.findRoute(e.getID(),target).edges
                        if len(path)>0: # target is reachable by the edge
                            mapdata.edge2target[e.getID()][target] = (True,True,list(path)) # update data structure to signal reachability
                            if forbid is None: # every edge reaching the target is considered valid
                                res.append(e)
                            else:
                                if forbid not in path: # include edge only if the path is not taking into account a forbidden edge
                                    res.append(e)
                        else: # target is unreachable, update data structure to signal unreachability
                            mapdata.edge2target[e.getID()][target] = (True,False,[])
                    else: # data structure is already updated
                        if mapdata.edge2target[e.getID()][target][1]: # if edge can reach the target
                            if forbid is None: # no edge is forbidden, include this neighbour
                                res.append(e)
                            else:
                                if forbid not in mapdata.edge2target[e.getID()][target][2]: # include edge only if the path is not taking into account a forbidden edge
                                    res.append(e)     
    return(res)

# utility method for turning index into string defining the type of algorithm to call, inputs are:
# - index to turn into string
def index2alg(index):
    if index==0:
        return 'e_dijkstra'
    if index==1:
        return 'e_astar'
    if index==2:
        return 'e_bfs'
    if index==3:
        return 'e_greedybfs'
    
# method for building paths according to index and target, inputs are: 
# - index to turn into string, 
# - target edge of the path to build, 
# - start edge of the path to build, 
# - data structure containing all static data related to the map, 
# - list of work-in-progress areas (optional)
def index2path(j,target_edge,e,mapdata,works=None):
    verbose = True # set to True for debugging
    route = []
    ext = None
    needforworks = False
    if verbose:
        print('edge '+str(e.getID()))
    if works:
        testext = build_path(mapdata,e.getID(),target_edge,index2alg(0)) # check if Dijkstra path includes wip areas
        if testext is not None:
            for ed in works:
                if ed in testext:
                    needforworks = True
                    break
            if needforworks: # check if avoiding works is actually helpful
                dpathlen = calculate_pathlen(testext,works,mapdata)
                avoiding_path = build_path(mapdata,e.getID(),target_edge,'e_dijkstra',forbidnode=works)
                dwpathlen = calculate_pathlen(avoiding_path,works,mapdata)
                if verbose:
                    print('dijkstra path length '+str(dpathlen))
                    print('dijkstra path avoiding works length '+str(dwpathlen))
                if dpathlen<dwpathlen:
                    needforworks = False
    if needforworks:
        # work areas detected in Dijkstra-based path, wip areas must be dodged
        if j<2:
            if verbose:
                print('works detected')
            ext = build_path(mapdata,e.getID(),target_edge,index2alg(j)) # create path with Dijkstra or A* including wip areas
            if ext is not None and verbose:
                print('edge '+str(e.getID())+' path based on '+str(index2alg(j))+' with length '+str(calculate_pathlen(ext,[], mapdata))+':'+str(ext))
        else: # for the other 2 behaviours
            if verbose:
                print('works detected')
            fnode = [] # list of nodes to forbid while building paths
            fnode.extend(works) # exclude wip areas
            index = j-2 # run Dijkstra or A* excluding wip areas
            ext = build_path(mapdata,e.getID(),target_edge,index2alg(index),forbidnode=fnode)
            if ext is not None and verbose:
                print('avoiding works')
    # if path has yet to be found or no wip areas detected in Dijkstra-based path
    if ext is None or not needforworks: # no work areas detected in general, explore as much as possible
        if j<2:
            if verbose:
                print('algorithm, '+str(index2alg(j)))
            ext = build_path(mapdata,e.getID(),target_edge,index2alg(j)) # create path with Dijkstra or A*
            if ext is not None and verbose:
                print('edge '+str(e.getID())+' path based on '+str(index2alg(j))+' with length '+str(calculate_pathlen(ext,[], mapdata))+':'+str(ext))
        else: # for the other 2 behaviours
            if ext is None or len(ext)==0: # no route was found using Dijkstra or A*
                ext = build_path(mapdata,e.getID(),target_edge,index2alg(j)) # use BFS or Greedy BFS
                if ext is not None and verbose:
                    print('edge '+str(e.getID())+' path based on '+str(index2alg(j))+' with length '+str(calculate_pathlen(ext,[], mapdata))+':'+str(ext))
    if ext is None:
        route = None
    else:
        if 'SUCC' not in ext: # add 'SUCC' string for emphasizing success of path search
            route.extend(ext)
    return route

# method for creating offline paths, inputs are: 
# - number of algorithms/behaviours for each target, 
# - data structure containing all static data related to the map, 
# - flag for taking into account wip areas (optional)
def create_paths(num_algs,mapdata,consider_works=False):
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")
    targets = mapdata.targets
    edge_list = mapdata.edgelist
    sumocfg_name = 'osm_'+str(mapdata.scenario)+'.sumocfg'
    sumoBinary = sumolib.checkBinary('sumo')

    sumoCmd = [sumoBinary, "-c", sumocfg_name,"--step-length","0.05"] #The last parameter is the step size, has to be small
    traci.start(sumoCmd)
    print("Starting SUMO...")

    works = mapdata.works if consider_works else {}
    i=0
    paths = {}
    for t in targets:
        dijkstrabased = {}
        target_edge = targets[t]
        for j in range(num_algs):
            for e in edge_list:
                if (e.getID() not in paths):
                    paths[e.getID()] = []
                route = index2path(j,target_edge,e,mapdata,works=works) # look for route given index and target
                if len(paths[e.getID()])<num_algs*len(targets):
                    if route is None: # no route found
                        paths[e.getID()].append(None)
                    else: # route found
                        paths[e.getID()].append(list(route))
                    valid_edges = valid_neighbors(e,mapdata,target_edge)
                    if len(valid_edges)>1 and j==0: # update data structure related to first edges in Dijkstra paths
                        dijkstrabased[e.getID()] = route[1]
            i+=1
    print("Paths generated!")
    traci.close()
    np.save('paths_'+str(mapdata.scenario)+'_'+str(num_algs)+('_wip' if consider_works else ''), paths)
    
# method for creating offline behaviours and paths, inputs are: 
# - number of algorithms/behaviours for each target, 
# - data structure containing all static data related to the map, 
# - flag for taking into account wip areas (optional)
def create_behaviours(num_algs,mapdata,consider_works=False):

    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")
    targets = mapdata.targets
    edge_list = mapdata.edgelist
    net = mapdata.net
    edge_dict = mapdata.edge_dict
    

    sumocfg_name = 'osm_'+str(mapdata.scenario)+'.sumocfg'
    sumoBinary = sumolib.checkBinary('sumo')

    sumoCmd = [sumoBinary, "-c", sumocfg_name,"--step-length","0.05"] #The last parameter is the step size, has to be small
    traci.start(sumoCmd)
    print("Starting SUMO...")

    behaviors = np.zeros((len(targets)*num_algs, len(edge_list), len(edge_list))) # initialize data structure for keeping all pmfs
    works = mapdata.works if consider_works else {}
    i=0
    paths = {}
    for t in targets:
        # pmfs = np.zeros((len(edge_list), len(edge_list))) # initialize pmfs for the target
        dijkstrabased = {}
        target_edge = targets[t]
        for j in range(num_algs):
            pmfs = np.zeros((len(edge_list), len(edge_list))) # initialize pmfs for the target and the j-th behaviour
            for e in edge_list:
                if (e.getID() not in paths): # if not in paths data structure, initialize it
                    paths[e.getID()] = []
                route = index2path(j,target_edge,e,mapdata,works=works) # look for path
                if len(paths[e.getID()])<num_algs*len(targets):
                    if route is None: # no route found
                        paths[e.getID()].append(None)
                    else: # route was found
                        paths[e.getID()].append(list(route))
                pmf = [0]*((len(edge_list))) # empty pmf for edge
                if route is None or len(route) ==0 : # edges are not connected 
                    for x in range(len(pmf)): # create empty pmf
                        pmf[x] = 0
                elif(len(route) == 1): # destination reached, put probability 1.0 on the only possible road
                    e_prime = e.getID()
                    pmf[edge_dict[e_prime]] = 1.0
                else:
                    e_prime = route[1] # e' is the edge just after e 

                    prob = 0.99 
                    pmf[edge_dict[e_prime]] = prob # put probability of choosing e' 0.99 as edge in the calculated path

                    valid_edges = valid_neighbors(e,mapdata,target_edge) # look for other valid neighbouring edges
                    for v in valid_edges:
                        if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                            pmf[edge_dict[v.getID()]] = (1-prob) / (len(valid_edges)-1) # assign to each of them the remaining probability
                    if len(valid_edges)>1 and j==0: # update data structure related to first edges in Dijkstra paths
                        dijkstrabased[e.getID()] = e_prime
                    
                pmf = np.array(pmf)
                        
                if np.sum(pmf) !=0: # normalize probabilities
                    pmf = pmf/np.sum(pmf)
                        
                pmfs[edge_dict[e.getID()]] = pmf # update pmfs data structure
            
            # after creating pmfs for each of the edges given the target and the index of the behaviours, update general behaviour data structure
            for k in range(len(pmfs)):
                behaviors[i,k] = behaviors[i,k]+pmfs[k]
                if np.sum(behaviors[i,k])>1: # check for debugging purposes
                    print('WARNING HERE')        
            i+=1
    print("Behaviors and paths generated!")
    traci.close()
    np.save('behaviors_'+str(mapdata.scenario)+'_'+str(num_algs)+('_wip' if consider_works else ''), behaviors)
    np.save('paths_'+str(mapdata.scenario)+'_'+str(num_algs)+('_wip' if consider_works else ''), paths)

# method for creating online behaviours and paths, inputs are: 
# - data structure containing all static data related to the map, 
# - number of algorithms/behaviours for each target, 
# - target edge of the vehicle calling this method, 
# - the list of edges for which to calculate paths and behaviours, 
# - data structure containing already built pmfs, 
# - data structure for checking if a behaviour is already created, 
# - list of wip areas, 
# - flag to check if behaviours must be updated or not (optional),
# - flag for coloring streets according to their pmf (optional)
def online_create_behaviours(mapdata,num_algs,target_edge,ss_edges,behaviors,behavior_created,works,toupdate=False,colorprobs=False):
    targets = mapdata.targets
    edge_list = mapdata.edgelist
    edge_dict = mapdata.edge_dict
    colorvalue = getIfromRGB(list(car_color(list(targets.keys())[list(targets.values()).index(target_edge)]))[0:3])
                            
    i = list(targets.values()).index(target_edge)*num_algs
    if colorprobs:
        for edg in edge_list:
            traci.edge.setParameter(edg.getID(),'color',0)
    dijkstrabased = {}
    paths = {}
    for j in range(num_algs):
        pmfs = np.zeros((len(edge_list), len(edge_list))) # initialize pmfs for the j-th behaviour
        for e in ss_edges:
            if (e.getID(),target_edge) not in behavior_created or toupdate: # if the behaviour is not already built or it needs to be updated
                if (e.getID() not in paths) or (j==0): # if not in paths data structure, initialize it
                    paths[e.getID()] = []
                route = index2path(j,target_edge,e,mapdata,works=works) # look for path
                if len(paths[e.getID()])<num_algs:
                    paths[e.getID()].append(list(route))
                pmf = [0]*((len(edge_list))) # empty pmf for edge
                if route is None or len(route) ==0 : # edges are not connected 
                    for x in range(len(pmf)): # create empty pmf
                        pmf[x] = 0
                elif(len(route) == 1): # destination reached, put probability 1.0 on the only possible road
                    e_prime = e.getID()
                    pmf[edge_dict[e_prime]] = 1.0
                else:
                    e_prime = route[1] # e' is the edge just after e 

                    prob = 0.99 
                    pmf[edge_dict[e_prime]] = prob # put probability of choosing e' 0.99 as edge in the calculated path

                    valid_edges = valid_neighbors(e,mapdata,target_edge) # look for other valid neighbouring edges
                    for v in valid_edges:
                        if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                            pmf[edge_dict[v.getID()]] = (1-prob) / (len(valid_edges)-1) # assign to each of them the remaining probability
                    if len(valid_edges)>1 and j==0:
                        dijkstrabased[e.getID()] = e_prime # update data structure related to first edges in Dijkstra paths
                    if colorprobs:
                        if j == 2:
                            for r in valid_edges:
                                traci.edge.setParameter(r.getID(),'color',colorvalue*pmf[edge_dict[r.getID()]])
                            
                    
                pmf = np.array(pmf)
                        
                if np.sum(pmf) !=0: # normalize probabilities
                    pmf = pmf/np.sum(pmf)
                        
                pmfs[edge_dict[e.getID()]] = pmf # update pmfs data structure
        # after creating pmfs for each of the edges, update general behaviour data structure
        for k in range(len(pmfs)):
            if toupdate and edge_list[k] in ss_edges: # clear previously built behaviour if update is needed
                behaviors[i,k] = np.zeros(len(edge_list))
            behaviors[i,k] = behaviors[i,k]+pmfs[k]
            su = np.sum(behaviors[i,k])
            if su>1:
                print('WARNING HERE') # check for debugging purposes   
        i+=1
    return behaviors,paths