# Module for gathering all path planning algorithms scripts

from queue import PriorityQueue
from queue import Queue
import math
import traci
from colors import alg_color
from colors import getIfromRGB
import time

ANIMATION = False # change to True if visual representation of how the algorithm works is required (warning, traCI must also be opened somewhere else in the script calling the following methods)
TSTEP = 0.01 # time step for path planning animation

# method to check if connection between two edges exists, inputs are:
# - the nodes at the extremes of the edges, 
# - a dictionary containing infos about the connections, 
# - an edge to use as placeholder if node1 is None (if the first edge of the path is being considered) (optional)
def connection_exists(node1,node2,node3,graphdict,connections,edge=None):
    try:
        edge1 = None
        if node1 is None:
            edge1=(edge,0)
        else:
            edge1 = graphdict[node1][node2]
        edge2 = graphdict[node2][node3]
        if edge2[0] in connections[edge1[0]] :
            return True # if connection between edge1 and edge2 exists, return True
    except:
        return False # if an edge has no connections or if any exception related to malformed data structures occurs, return False
    return False # if no connection was found, return False

# "gate" method for requiring path building, inputs are:
# - data structure containing all static data related to the map,
# - node/edge from which to start building the path,
# - node/edge that must be reached by the path,
# - a string defining the path planning algorithm to call,
# - a list containing nodes that must be avoided by the path planning algorithm (optional),
# - edge for aiding in checking existing connections (optional)
def build_path(mapdata,start,goal,type,forbidnode=None,edge=None):
    graphdict = mapdata.graphdict
    connections = mapdata.connections
    graphmap = mapdata.graphmap
    edgegraph = mapdata.edgegraph
    if type=='bfs':
        return bfs(graphdict,start,goal,connections,edge)
    elif type=='dijkstra':
        return dijkstra(graphdict,start,goal,forbidnode,connections,edge)
    elif type=='greedybfs':
        return greedybfs(graphdict,start,goal,graphmap,connections,edge)
    elif type=='astar':
        return astar(graphdict,start,goal,graphmap,forbidnode,connections,edge)
    elif type=='e_bfs':
        return edge_bfs(start,goal,edgegraph,forbidnode)
    elif type=='e_greedybfs':
        return edge_greedybfs(mapdata,start,goal,forbidnode)
    elif type=='e_dijkstra':
        return edge_dijkstra(mapdata,start,goal,forbidnode)
    elif type=='e_astar':
        return edge_astar(mapdata,start,goal,forbidnode)
    return None

# method for calling BFS algorithm with focus on the nodes of the graph, inputs are: 
# - data structure containing data about the road network graph, 
# - start node of the path to find, 
# - goal node of the path to find, 
# - dictionary containing infos about the connections between edges,
# - edge for aiding in checking existing connections (optional)
def bfs(graphdict,start,goal,connections,edge=None):
    if start == goal:
        return ['SUCC']
    frontier = PriorityQueue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None
    goal_found = False
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if dests not in came_from and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                frontier.put(dests)
                came_from[dests] = current
    if goal_found:
        path = []
        path.append(graphdict[came_from[goal]][goal][0])
        current = came_from[goal]
        while current!=start:
            path.append(graphdict[came_from[current]][current][0])
            current = came_from[current]
        path.reverse()
        return path
    else:
        return None

# method for calling Dijkstra algorithm with focus on the nodes of the graph, inputs are: 
# - data structure containing data about the road network graph, 
# - start node of the path to find, 
# - goal node of the path to find, 
# - a list containing nodes that must be avoided by the path planning algorithm (optional),
# - dictionary containing infos about the connections between edges (optional),
# - edge for aiding in checking existing connections (optional)
def dijkstra(graphdict,start,goal,forbidnode=None,connections=None,edge=None):
    frontier = PriorityQueue()
    frontier.put(start,0)
    came_from = {}
    came_from[start] = None
    cost_so_far = {}
    cost_so_far[start] = 0
    i = 0
    goal_found = False
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if (dests!=forbidnode and current == start) or current!=start:
                new_cost = float(cost_so_far[current])+float(graphdict[current][dests][1])
                if (dests not in cost_so_far or new_cost<cost_so_far[dests]) and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                    frontier.put(dests,new_cost)
                    i += 1
                    came_from[dests] = current
                    cost_so_far[dests] = new_cost
    if goal_found:
        path = []
        if came_from[goal] is None:
            return ['SUCC']
        path.append(graphdict[came_from[goal]][goal][0])
        current = came_from[goal]
        while current!=start:
            path.append(graphdict[came_from[current]][current][0])
            current = came_from[current]
        path.reverse()
        return path
    else:
        return None

# method for de-coupling heuristic between codes of Greedy BFS and A* algorithms and how the heuristic is integrated (in this case the heuristic is euclidean distance between nodes), inputs are: 
# - first node for calculation of euclidean distance,
# - second node for calculation of euclidean distance,
# - data structure containing coordinates data about the road network graph
def heuristic(n0,n1,graphmap):
    n0x = float(graphmap[n0][0])
    n1x = float(graphmap[n1][0])
    n0y = float(graphmap[n0][1])
    n1y = float(graphmap[n1][1])
    hvalue = math.sqrt((n0x-n1x)**2+(n0y-n1y)**2) # euclidean distance between node coordinates
    return hvalue

# method for calling Greedy BFS algorithm with focus on the nodes of the graph, inputs are: 
# - data structure containing data about the road network graph, 
# - start node of the path to find, 
# - goal node of the path to find, 
# - data structure containing coordinates data about the road network graph,
# - dictionary containing infos about the connections between edges (optional),
# - edge for aiding in checking existing connections (optional)
def greedybfs(graphdict,start,goal,graphmap,connections,edge=None):
    if start == goal:
        return ['SUCC']
    frontier = PriorityQueue()
    frontier.put(start,0)
    came_from = {}
    came_from[start] = None
    goal_found = False
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if dests not in came_from and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                frontier.put(dests,heuristic(dests,goal,graphmap))
                came_from[dests] = current
    if goal_found:
        path = []
        if came_from[goal] is None:
            return ['SUCC']
        path.append(graphdict[came_from[goal]][goal][0])
        current = came_from[goal]
        while current!=start:
            path.append(graphdict[came_from[current]][current][0])
            current = came_from[current]
        path.reverse()
        return path
    else:
        return None

# method for calling A* algorithm with focus on the nodes of the graph, inputs are: 
# - data structure containing data about the road network graph, 
# - start node of the path to find, 
# - goal node of the path to find, 
# - data structure containing coordinates data about the road network graph,
# - a list containing nodes that must be avoided by the path planning algorithm (optional),
# - dictionary containing infos about the connections between edges (optional),
# - edge for aiding in checking existing connections (optional)
def astar(graphdict,start,goal,graphmap,forbidnode=None,connections=None,edge=None):
    frontier = PriorityQueue()
    frontier.put(start,0)
    came_from = {}
    came_from[start] = None
    cost_so_far = {}
    cost_so_far[start] = 0
    i = 0
    goal_found = False
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if (dests!=forbidnode and current == start) or current!=start:
                new_cost = float(cost_so_far[current])+float(graphdict[current][dests][1])
                if (dests not in cost_so_far or new_cost<cost_so_far[dests]) and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                    frontier.put(dests,new_cost+heuristic(dests,goal,graphmap))
                    i += 1
                    came_from[dests] = current
                    cost_so_far[dests] = new_cost
    if goal_found:
        path = []
        if came_from[goal] is None:
            return ['SUCC']
        path.append(graphdict[came_from[goal]][goal][0])
        current = came_from[goal]
        while current!=start:
            path.append(graphdict[came_from[current]][current][0])
            current = came_from[current]
        path.reverse()
        return path
    else:
        return None

# method for calling BFS algorithm with focus on the edges of the graph, inputs are: 
# - start edge of the path to find, 
# - goal edge of the path to find, 
# - dictionary containing infos about the graph built between edges,
# - a list containing nodes that must be avoided by the path planning algorithm (optional)
def edge_bfs(start,goal,edgegraph,forbidnode=None):
    if start == goal:
        return ['SUCC']
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None
    goal_found = False
    colorv = getIfromRGB(list(alg_color('e_bfs'))[0:3])
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break               
        if ANIMATION and not goal_found:
            traci.edge.setParameter(current,'color',colorv)
            time.sleep(TSTEP)
            traci.simulationStep()
        for dests in edgegraph[current]:
            if dests not in came_from and (forbidnode is None or (forbidnode is not None and dests not in forbidnode)):
                frontier.put(dests)
                came_from[dests] = current
    if goal_found:
        path = []
        current = goal
        while current!=start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    else:
        return None
    
# method for calling Greedy BFS algorithm with focus on the edges of the graph, inputs are: 
# - data structure containing all static data related to the map,
# - start edge of the path to find, 
# - goal edge of the path to find, 
# - a list containing nodes that must be avoided by the path planning algorithm (optional)
def edge_greedybfs(mapdata,start,goal,forbidnode=None):
    edgegraph = mapdata.edgegraph
    graphmap = mapdata.graphmap
    net = mapdata.net
    if start == goal:
        return ['SUCC']
    frontier = PriorityQueue()
    frontier.put((0,start))
    came_from = {}
    came_from[start] = None
    goal_found = False
    colorv = getIfromRGB(list(alg_color('e_greedybfs'))[0:3])
    while not frontier.empty():
        current = frontier.get()[1]
        if current == goal:
            goal_found = True
            break
        if ANIMATION and not goal_found:
            traci.edge.setParameter(current,'color',colorv)
            time.sleep(TSTEP)
            traci.simulationStep()
        for dests in edgegraph[current]:
            if dests not in came_from and (forbidnode is None or (forbidnode is not None and dests not in forbidnode)):
                frontier.put((heuristic(net.getEdge(dests).getToNode().getID(),net.getEdge(goal).getToNode().getID(),graphmap),dests))
                came_from[dests] = current
    if goal_found:
        path = []
        current = goal
        while current!=start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    else:
        return None
    
# method for calling Dijkstra algorithm with focus on the edges of the graph, inputs are: 
# - data structure containing all static data related to the map,
# - start edge of the path to find, 
# - goal edge of the path to find, 
# - a list containing nodes that must be avoided by the path planning algorithm (optional)
def edge_dijkstra(mapdata,start,goal,forbidnode=None):
    if start == goal:
        return ['SUCC']
    edgegraph = mapdata.edgegraph
    net = mapdata.net
    frontier = PriorityQueue()
    frontier.put((0,start))
    came_from = {}
    came_from[start] = None
    cost_so_far = {}
    cost_so_far[start] = edge_cost(mapdata,start)
    goal_found = False
    colorv = getIfromRGB(list(alg_color('e_dijkstra'))[0:3])
    while not frontier.empty():
        popel = frontier.get()
        current = popel[1]
        if current == goal:
            goal_found = True
            break
        if ANIMATION and not goal_found:
            traci.edge.setParameter(current,'color',colorv)
            time.sleep(TSTEP)
            if not goal_found:
                traci.simulationStep()
        for dests in edgegraph[current]:
            if forbidnode is None or (forbidnode is not None and dests not in forbidnode):
                new_cost=float(cost_so_far[current])+edge_cost(mapdata,dests)
                if dests not in cost_so_far or new_cost<cost_so_far[dests]:
                    frontier.put((new_cost,dests))
                    came_from[dests] = current
                    cost_so_far[dests] = new_cost
    if goal_found:
        path = []
        current = goal
        while current!=start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    else:
        return None
    
# method for calling A* algorithm with focus on the edges of the graph, inputs are: 
# - data structure containing all static data related to the map,
# - start edge of the path to find, 
# - goal edge of the path to find, 
# - a list containing nodes that must be avoided by the path planning algorithm (optional)
def edge_astar(mapdata,start,goal,forbidnode=None):
    if start == goal:
        return ['SUCC']
    edgegraph = mapdata.edgegraph
    net = mapdata.net
    graphmap = mapdata.graphmap
    frontier = PriorityQueue()
    frontier.put((0,start))
    came_from = {}
    came_from[start] = None
    cost_so_far = {}
    cost_so_far[start] = edge_cost(mapdata,start)
    goal_found = False
    colorv = getIfromRGB(list(alg_color('e_astar'))[0:3])
    while not frontier.empty():
        current = frontier.get()[1]
        if current == goal:
            goal_found = True
            break
        if ANIMATION and not goal_found:
            traci.edge.setParameter(current,'color',colorv)
            time.sleep(TSTEP)
            traci.simulationStep()
        for dests in edgegraph[current]:
            if forbidnode is None or (forbidnode is not None and dests not in forbidnode):
                new_cost=float(cost_so_far[current])+edge_cost(mapdata,dests)
                if dests not in cost_so_far or new_cost<cost_so_far[dests]:
                    frontier.put((new_cost+heuristic(net.getEdge(dests).getToNode().getID(),net.getEdge(goal).getToNode().getID(),graphmap),dests))
                    came_from[dests] = current
                    cost_so_far[dests] = new_cost
    if goal_found:
        path = []
        current = goal
        while current!=start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    else:
        return None

# method for de-coupling heuristic between codes of algorithms and how the cost is integrated, inputs are: 
# - data structure containing all static data related to the map,
# - edge of which cost must be evaluated
def edge_cost(mapdata,edge):
    net = mapdata.net
    return float(net.getEdge(edge).getLength()/net.getEdge(edge).getSpeed()) # use travel time of edge using static data
    # return traci.edge.getTraveltime(edge) # use travel time calculated by TraCI (same values as the static data, it slows down computation because of TraCI requests)
    