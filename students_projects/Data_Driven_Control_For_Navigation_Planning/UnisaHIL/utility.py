import optparse
import traci
import numpy as np


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

def add_agents(n_agents, departs, start):
    #For each agent, create a route with the starting edge, add agent
    for i in range(n_agents):
        routeId = 'route_agent.' + str(i)
        traci.route.add(routeId, (start[i], start[i]))
        traci.vehicle.add("agent."+str(i), routeId, depart=departs[i])
        traci.vehicle.setColor("agent." + str(i), color=(255, 0, 0, 255))


def add_foes(n_foes, departs, start, finish, parkings, lay_off):
    #Create foe route using findRoute, assign parking space
    for i in range(n_foes):
        routeId = 'route_foe.'+str(i)
        traci.route.add(routeId, traci.simulation.findRoute(start[i], finish[i]).edges)
        foeId = 'foe.'+str(i)
        traci.vehicle.add(foeId, routeId, depart = departs[i])
        traci.vehicle.setParkingAreaStop(foeId, parkings[finish[i]], duration = lay_off)
        traci.vehicle.setColor(foeId, color=(0,0,255,255))


def neighbor_edges(n_edge, edge_dict, net):
    #Find neighbor edges
    edge = net.getEdge(edge_dict[n_edge]) #Get the edge
    res = []
    #get all the incoming and outgoing edges of the end node
    n1 = edge.getToNode()
    for e in (n1.getIncoming()+n1.getOutgoing()):
        ed = [x for x in edge_dict if edge_dict[x] == e.getID() and x != n_edge]
        if ed != []:
            res.append(ed[0])
    #get all the incoming and outgoing edges of the starting node
    n2 = edge.getFromNode()
    for e in (n2.getIncoming()+n2.getOutgoing()):
        ed = [x for x in edge_dict if edge_dict[x] == e.getID() and x != n_edge]
        if ed != []:
            res.append(ed[0])
    return(res)

def getEdgeIndex(id, edge_dict):
    #get index of edge from its ID
    return([x for x in edge_dict if edge_dict[x] == id][0])

def valid_neighbors(n_edge, edge_dict, net):
    #get only valid turns for the given edge
    edge = net.getEdge(edge_dict[n_edge])
    res = []
    n1 = edge.getToNode()
    for e in n1.getOutgoing():
        ed = [x for x in edge_dict if edge_dict[x] == e.getID() and x != n_edge]
        if ed != []:
            res.append(ed[0])
    return(res)

def state_space(n_node, tHor, edge_dict, net):
    #State space for a given horizon: iteratively check every neighbor and add it to the state space
    stsp = [n_node]
    for t in range(tHor):
        newSpace = stsp.copy()
        for n in stsp:
            for x in neighbor_edges(n, edge_dict,net):
                if x not in newSpace:
                    newSpace.append(x)
        stsp = newSpace
    return(np.array(stsp))

def change_route_color(edge):
    traci.gui.toggleSelection(objID=edge, objType="edge")
    return
    
def update_parking_reward(parkings,edge_list,edge_dict, number_lots):
    r1 = [0]*len(edge_list)
    for p in parkings:
        parkId = parkings[p] #Get parking name
        edge = [x for x in edge_dict if edge_dict[x] == p][0] #get index of corresponding edge
        if traci.parkingarea.getVehicleCount(parkId) < number_lots[p]:
            r1[edge] = 100
        else:
            r1[edge] = 0 
    return(np.array(r1))

def update_pedestrians_reward(edge_list, parkings, step):
    r = [0]*len(edge_list)
    if step>0:
        for i in range(len(edge_list)):
            if edge_list[i] not in parkings:
                lane_ID = edge_list[i].getID() + "_0" 
                density = (len(traci.edge.getLastStepPersonIDs(edge_list[i].getID())))/traci.lane.getLength(lane_ID) 
                if (density*100 > 1):
                    r[i] = density 
                else: 
                    r[i] = 100 
    return(np.array(r))


def update_traffic_reward(edge_list, parkings, step):
    r2 = [0]*len(edge_list)
    if step>0:
        for i in range(len(edge_list)):
            if edge_list[i] not in parkings:
                lane_ID = edge_list[i].getID() + "_0" 
                density = (traci.edge.getLastStepVehicleNumber(edge_list[i].getID()))/traci.lane.getLength(lane_ID)
                if (density*100 > 1):
                    r2[i] = density 
                else: 
                     r2[i] = 100 
    return(np.array(r2))
        
def getFreeParking(parkings, number_lots):
    #Get free parking lots
    return([p for p in parkings if traci.parkingarea.getVehicleCount(parkings[p]) < number_lots[p]])
    
def rerouteFoe(foeID, parkings, number_lots):
    #Reroute foe to random free parking lot
    l = getFreeParking(parkings, number_lots)
    if len(l) == 0: #Failsafe if no parking lot is free
        edge='-906615585'
    else:
        edge = np.random.choice(l)
    #Set target and parking area
    traci.vehicle.changeTarget(foeID, edge)
    traci.vehicle.setParkingAreaStop(foeID, parkings[edge])
    
def rerouteAgent(agentI, parkings, agents, crowds, number_lots):
    l = getFreeParking(parkings, number_lots)
    #we have to iterate over each possible parking as the target needs to be set by hand
    # targets = ['terminal','biblioteca', 'multipiano']
    if '-587489968#0' in l:
        agents[agentI].id_goal = 0 
        crowds[agentI].nTarget = 0 
        return '-587489968#0'
    elif '-906615585' in l:
        agents[agentI].id_goal = 1 
        crowds[agentI].nTarget = 1
        return '-906615585'
    else:
        agents[agentI].id_goal = 2 
        crowds[agentI].nTarget = 2 
        return '298563412'
    

def computeReward(pR, tR, pedR):
    alpha = 100
    beta = 10
    gamma = 3
    return alpha*pow(pR,2) + beta*pow(tR,2) + gamma*pow(pedR,2)
    