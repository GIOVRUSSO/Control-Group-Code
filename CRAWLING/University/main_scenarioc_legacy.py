import copy
import os
import pickle
import sys
import tweepy

import sumolib
import traci

import numpy as np

from agent import Agent
from crowdsourcing import Crowdsourcing

from sumolib.net import readNet


###To set the step length modify sumo.cmd  option --step-length


parkings = {'-906615585':'biblioteca', '-587489968#0':'terminal', '298563412':'multipiano'}
exits = ['-147391538#1','-122726134#0','442152904'] #Leaving this here in case it is useful for a scenario

sumo_net_path = "sumo_files/osm3.net.xml"
sumo_add_path = "sumo_files/add.add.xml"

#Initialiwing log lists
logs = []
foe_tR = [0]*90001
agent_tR = [0]*90001
total_cars = 150

#Loading agent and foe info
#agentArray = np.load('agent.npy')
#foeArray = np.load('foe.npy')

#Connected cars info (start, departure time and target)
agSt = ['62166872']*2
agDep = [0,30] #The second car leaves 30 secs after the first
agGoals = [4]*2

#No uncontrolled cars
foeSt = []
foeDep = []
foeFin = []


#Twitter developper account information
consumer_key = 
consumer_secret = 
access_token = 
access_token_secret = 

auth = tweepy.OAuthHandler(consumer_key, consumer_secret)
auth.set_access_token(access_token, access_token_secret)
api = tweepy.API(auth)
username='EGarrabe' #Account


today = '2022-07-29'
def parse_tweet(stri, date):
    if '#sumo_experiment' in stri and 'blocked' in stri:
        if date.split()[0]==today:
            sp = stri.split()
            i = sp.index('blocked')
            return([sp[i-3],sp[i-2],sp[i-1]],'blocked')

def add_agents(n_agents, departs, start):
    #For each agent, create a route with the starting edge, add agent
    for i in range(n_agents):
        routeId = 'route_agent.' + str(i)
        traci.route.add(routeId, (start[i], start[i]))
        traci.vehicle.add("agent."+str(i), routeId, depart=departs[i])
        traci.vehicle.setColor("agent." + str(i), color=(255, 0, 0, 255))


def add_foes(n_foes, departs, start, finish):
    #Create foe route using findRoute, assign parking space
    for i in range(n_foes):
        routeId = 'route_foe.'+str(i)
        traci.route.add(routeId, traci.simulation.findRoute(start[i], finish[i]).edges)
        foeId = 'foe.'+str(i)
        traci.vehicle.add(foeId, routeId, depart = departs[i])
        traci.vehicle.setParkingAreaStop(foeId, parkings[finish[i]], duration = lay_off)
        traci.vehicle.setColor(foeId, color=(0,0,255,255))


def neighbor_edges(n_edge):
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

def valid_neighbors(n_edge):
    #get only valid turns for the given edge
    edge = net.getEdge(edge_dict[n_edge])
    res = []
    n1 = edge.getToNode()
    for e in n1.getOutgoing():
        ed = [x for x in edge_dict if edge_dict[x] == e.getID() and x != n_edge]
        if ed != []:
            res.append(ed[0])
    return(res)

def state_space(n_node, tHor):
    #State space for a given horizon: iteratively check every neighbor and add it to the state space
    stsp = [n_node]
    for t in range(tHor):
        newSpace = stsp.copy()
        for n in stsp:
            for x in neighbor_edges(n):
                if x not in newSpace:
                    newSpace.append(x)
        stsp = newSpace
    return(np.array(stsp))

def change_route_color(route):
    for edge in route:
        traci.gui.toggleSelection(objID=edge, objType="edge")
    return
    
def update_parking_reward():
    #High reward for non-full lots, negative reward for full ones
    r1 = [0]*len(edge_list)
    for p in parkings:
        parkId = parkings[p] #Get parking name
        edge = [x for x in edge_dict if edge_dict[x] == p][0] #get index of corresponding edge
        if traci.parkingarea.getVehicleCount(parkId) < 50:
            r1[edge] = 100
        else:
            r1[edge] = -10
    return(np.array(r1))

badInds = [189] #Index of the highway ramp
def setRiskReward():
    r3 = [0]*len(edge_list)
    for i in badInds:
        traci.edge.setMaxSpeed(edge_list[i].getID(), 0.2) #Set low speed
        r3[i] = -100 #set negative reward
    traci.gui.toggleSelection(objID=edge_list[badInds[0]].getID(), objType="edge")
    return(np.array(r3))

def getEdgeIndex(id):
    #get index of edge from its ID
    return([x for x in edge_dict if edge_dict[x] == id][0])
        
def getFreeParking():
    #Get free parking lots
    return([p for p in parkings if traci.parkingarea.getVehicleCount(parkings[p]) < 50])
    
def rerouteFoe(foeID):
    #Reroute foe to random free parking lot
    l = getFreeParking()
    if len(l) == 0: #Failsafe if no parking lot is free
        edge='-906615585'
    else:
        edge = np.random.choice(l)
    #Set target and parking area
    traci.vehicle.changeTarget(foeID, edge)
    traci.vehicle.setParkingAreaStop(foeID, parkings[edge])
    
def rerouteAgent(agentI):
    l = getFreeParking()
    #we have to iterate over each possible parking as the target needsto be set by hand
    if '-587489968#0' in l:
        agents[agentI].id_goal = 3
        crowds[agentI].nTarget = 3
    elif '-906615585' in l:
        agents[agentI].id_goal = 4
        crowds[agentI].nTarget = 4
    else:
        agents[agentI].id_goal = 5
        crowds[agentI].nTarget = 5    
    
if __name__ == '__main__':
        
    n_agents = len(agSt)
    n_foes = len(foeSt)
    
    lay_off = 100000.0 #Very long layoff

    agents = []  # stuttura che mantiene le istanze degli agenti
    crowds = []  # struttura che mantiene le istanze alla classe crowdsoucing degli agenti

    #add agents and controllers
    for i in range(n_agents):
        agt = Agent(sumo_add_path, agSt[i], agGoals[i])
        agents.append(agt)
        crowds.append(Crowdsourcing(agents[i]))

    #This error means there is a small installation issue, solution can be found online
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")

    #Choose between these lines for GUI or not
    sumoBinary = sumolib.checkBinary('sumo-gui')
    #sumoBinary = sumolib.checkBinary('sumo')

    sumoCmd = [sumoBinary, "-c", r"sumo_files/osm.sumocfg","--step-length","0.05"] #The last parameter is the step size, has to be small
    traci.start(sumoCmd)
    print("Starting SUMO...")
    
    
    net = readNet(r"sumo_files/osm.net.xml") #read road network
    edge_list = net.getEdges()
    #Dicionnary of list IDs
    edge_dict = {}
    for i in range(len(edge_list)):
        edge_dict[i] = edge_list[i].getID()

    #Add agents and foes to SUMO
    add_agents(n_agents,agDep,agSt)
    add_foes(n_foes, foeDep, foeSt, foeFin)

    step = 0

    #Indexes of the blocked edges
    badEdges = [edge_list[i].getID() for i in badInds]

    #List of each agent's last edge (for now their beginning edge)
    past_edge = [traci.vehicle.getRoadID('agent.'+str(i)) for i in range(n_agents)]
    
    #Reward
    r = [0]*len(edge_list)

    done = False #Parser flag

    #Begin simulation
    while step < 90001:
        if not done and step%20==0: #We turn off the parser once it finds an issue to obtain a smoother video. We query it every second (20 steps)
            tweet_list= api.user_timeline(screen_name=username, count=1)
            tweet = tweet_list[0]
            res = parse_tweet(tweet.text, str(tweet.created_at))

            if res != None and res[1]=='blocked': #Update the reward if the relevant information is in the tweet
                r = setRiskReward()
                done = True

        for i in range(n_agents):
            if "agent." + str(i) in traci.vehicle.getIDList(): #Could be replaced by 'for id in IDList'
                
                current_edge = traci.vehicle.getRoadID('agent.' + str(i)) #get current edge

                #if current_edge in badEdges: #Log if we are out of place
                #    agent_tR[step] = agent_tR[step] + 1                

                crowds[i].reward = r #Send the reward to the agent's controller
                
                if past_edge[i] != current_edge and current_edge[
                    0] != ":":  #: to avoid calculating the route to an intersection
                    state = [x for x in edge_dict if edge_dict[x] == current_edge][0] #get state index
                    
                    if current_edge in parkings and traci.parkingarea.getVehicleCount(parkings[current_edge])<50: #Park if we can
                        traci.vehicle.setParkingAreaStop('agent.' + str(i), parkings[current_edge],
                                                     duration=lay_off)
                    else: #DM loop
                        
                        new_state = crowds[i].max_sampling_DM(state, 5, state_space(state, 5), r)
                    
                        prox_edge = edge_dict[new_state]
                    
                        traci.vehicle.changeTarget('agent.'+str(i), prox_edge)
                    past_edge[i] = current_edge #Update edge

                if current_edge in parkings and traci.parkingarea.getVehicleCount(parkings[current_edge])==50 and agents[i].id_goal==6: #If we need to be rerouted (the last condition is a workaround to replace an explicit rerouting flag)
                    
                    rerouteAgent(i)
                    new_state = crowds[i].max_sampling_DM(state, 5, state_space(state, 5), r) #Redo DM loop after rerouting
                    prox_edge = edge_dict[new_state]
                    traci.vehicle.changeTarget('agent.'+str(i), prox_edge)

        for i in range(n_foes):
            foeID = 'foe.' + str(i)
            if foeID in traci.vehicle.getIDList():
                #foesInSim = foesInSim + 1
                #foeSteptR = foeSteptR -2*traci.edge.getLastStepVehicleNumber(traci.vehicle.getRoadID(foeID))
                if traci.vehicle.getRoadID(foeID) == foeFin[i]: #Reroute if we arrived at a full parking lot
                    if traci.parkingarea.getVehicleCount(parkings[traci.vehicle.getRoadID(foeID)]) > 49:
                        rerouteFoe(foeID)
                #if traci.vehicle.getRoadID(foeID) in badEdges: #Log if we are on a prohibited edge
                #    foe_tR[step] = foe_tR[step] + 1
        
        traci.simulationStep()
        
        step += 1
        
       
        
    traci.close()
