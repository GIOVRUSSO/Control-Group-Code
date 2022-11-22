import optparse
from random import sample
import traci
import numpy as np
from operator import itemgetter


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

def change_route_color(route):
    for edge in route:
        traci.gui.toggleSelection(objID=edge, objType="edge")
    return
    
def update_parking_reward(parkings,edge_list,edge_dict, number_lots):
    # Remark: the reward fuction is alpha*pow(pR,2) + beta*pow(tR,2) + gamma*pow(pedR,2)
    # pR is the component referred to parking areas. 
    #High reward for non-full lots, zero for full ones
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
    # Remark: the reward fuction is alpha*pow(pR,2) + beta*pow(tR,2) + gamma*pow(pedR,2)
    # pedR is the component referred to pedestrians. 
    #High reward for low-density (or density equal to 0) edges
    r = [0]*len(edge_list)
    if step>0:
        for i in range(len(edge_list)):
            if edge_list[i] not in parkings:
                pp_list = traci.edge.getLastStepPersonIDs(edge_list[i].getID())
                pp_number = len(pp_list)
                if pp_number ==0:
                    density =0
                else:
                    lane_ID = traci.person.getLaneID(pp_list[0])
                    lane_length = traci.lane.getLength(lane_ID) 
                    density = pp_number/lane_length 
                if (density*100 > 1):
                    r[i] = density #The higher the density, the lower the reward (PS: It was found that the density values were in the order of 10^-2.)
                else: 
                    r[i] = 100 #Lower density (or no) edges will get higher rewards
    return(np.array(r))


def update_traffic_reward(edge_list, parkings, step): 
    # Remark: the reward fuction is alpha*pow(pR,2) + beta*pow(tR,2) + gamma*pow(pedR,2)
    # tR is the component referred to traffic. 
    #High reward for low-density (or density equal to 0) edges
    r2 = [0]*len(edge_list)
    if step>0:
        for i in range(len(edge_list)):
            if edge_list[i] not in parkings:
                veh_number = traci.edge.getLastStepVehicleNumber(edge_list[i].getID())
                if len (traci.edge.getLastStepVehicleIDs(edge_list[i].getID())) == 0:
                    density =0
                else: 
                    veh_id = traci.edge.getLastStepVehicleIDs(edge_list[i].getID())[0] # first item in the list
                    lane_ID = traci.vehicle.getLaneID(veh_id)
                    lane_length = traci.lane.getLength(lane_ID)
                    density = (veh_number)/lane_length
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
        edge= '-1001198066'
    else:
        edge = np.random.choice(l)
    #Set target and parking area
    traci.vehicle.changeTarget(foeID, edge)
    traci.vehicle.setParkingAreaStop(foeID, parkings[edge])
    
def rerouteAgent(agentI, parkings, agents, crowds, number_lots, parking_goals, parking_coordinates):
    l = getFreeParking(parkings, number_lots)
    #we have to iterate over each possible parking as the target needs to be set by hand

    a_goal = agents[agentI].id_goal
    desired_parking = parking_goals[a_goal] #if we are here, it means that desired_parking is full
    desired_coordinates = parking_coordinates[desired_parking]

    if desired_parking == "673737658#3" and "673737658#6" in l:
        agents[agentI].id_goal = 16
        crowds[agentI].nTarget = 16
        return "673737658#6"

    distances = []
    for freep in l:
        p_coord = parking_coordinates[freep]
        distances.append([freep, traci.simulation.getDistance2D(desired_coordinates[0], desired_coordinates[1], p_coord[0], p_coord[1], isGeo=False, isDriving=True)])

    dist_sorted = sorted(distances, key=itemgetter(1))
    dist_edges = []
    if len(dist_sorted) >0:
        for dd in dist_sorted:
            dist_edges.append(dd[0])

    stopp = int(0.2*len(dist_edges))
    if stopp !=0:
        subdist_edges = dist_edges[:stopp]
        randomp = sample(subdist_edges, 1)
        # Random parking space between the nearest ones
    else:
        randomp = sample(dist_edges, 1)
    
    idgoal = [i for i in parking_goals if parking_goals[i]== randomp[0]]
    agents[agentI].id_goal = idgoal[0]
    crowds[agentI].nTarget = idgoal[0]
    return randomp[0]


def computeReward(pR, tR, pedR):
    # Remark: the reward fuction is alpha*pow(pR,2) + beta*pow(tR,2) + gamma*pow(pedR,2)
    # alpha, beta and gamma are chosen accordingly to priority: we give more priority to available parking areas, then less busy roads, then less crowded sidewalks.
    alpha = 100
    beta = 10
    gamma = 3
    return alpha*pow(pR,2) + beta*pow(tR,2) + gamma*pow(pedR,2)
