# Module containing all classes and methods for management of static data of the maps

from node_file_parser import parse_file_for_nodes
from node_file_parser import parse_file_for_work
from node_file_parser import parse_file_for_checkpoints
import traci
import os
import sumolib

# method for building all useful data structures from the net.xml file of the desired map, inputs are:
# - scenario name (actually map name)
def build_graph(scenario):
    net = sumolib.net.readNet(str(scenario)+'.net.xml')
    nodes = traci.junction.getIDList()
    graphdict = {}
    graphmap = {}
    # build data structure representing graph based on the nodes of the network
    for n in nodes:
        graphdict[n] = {}
        try:
            graphmap[n] = net.getNode(n).getCoord()
            for neighbour in net.getNode(n).getNeighboringNodes(outgoingNodes=True,incomingNodes=False):
                for edge in net.getNode(n).getOutgoing():
                    if net.getEdge(edge.getID()).getToNode().getID() == neighbour.getID() and net.getEdge(edge.getID()).allows('passenger'):
                        graphdict[n][neighbour.getID()] = edge.getID(),edge.getLength()
        except:
            pass
    connects = sumolib.xml.parse_fast(str(scenario)+'.net.xml','connection',['from','to','dir'])
    connections = {}
    # build data structure containing infos about the connections between edges
    for c in connects:
        key = c[0]
        if key not in connections:
            connections[key] = {}
        val = c[1]
        if net.getEdge(val).allows('passenger'):
            connections[key][val] = c[2]
    edgegraph = {}
    # build data structure representing graph based on the edges of the network
    edges = net.getEdges()
    for e in edges:
        edid = e.getID()
        if net.getEdge(edid).allows('passenger'):
            edgegraph[edid] = []
            if edid in connections:
                edgegraph[edid].extend(list(connections[edid].keys()))
    return graphdict, graphmap, net, connections, edgegraph

# Roundabout class for keeping together data about roundabouts in the map
class Roundabout():
    
    def __init__(self,id,roads,mapdata=None,exits=None):
        self.id = id # identifier for the roundabout
        self.roads = roads # list of roads inside the roundabout
        self.exits = [] if exits is None else exits # exits of the roundabout
        if exits is None:
            net = mapdata.net
            for edge in self.roads:
                o_outs = net.getEdge(edge).getOutgoing()
                outs = []
                if len(o_outs)>1:
                    for e in o_outs:
                        if net.getEdge(e.getID()).allows('passenger'):
                            outs.append(e)
                    for e in outs:
                        if e.getID() not in self.roads:
                            self.exits.append(e.getID())
    
    # method for defining the order in which the roundabout exits are seen in a path, inputs are:
    # - instance of Roundabout,
    # - data structure containing all static data related to the map,
    # - path used for reference to define the order of the exits
    def exit_order(self,mapdata,path):
        net = mapdata.net
        starting_edge = None
        exit_id = 1
        for e in path:
            if e in self.roads and starting_edge is None: # initially look for the first edge in the path that also belongs to the roundabout
                starting_edge = e
            if starting_edge is not None and e in self.roads: # check neighbours of the edges in the roundabout and in the path
                o_outs = net.getEdge(e).getOutgoing()
                outs = []
                if len(o_outs)>1:
                    for e in o_outs:
                        if net.getEdge(e.getID()).allows('passenger'):
                            outs.append(e)
                    if len(outs)>1: # if the edge has neighbours
                        for e in outs:
                            if e.getID() in self.exits: # if neighbour is one of the exits
                                if e.getID() not in path: # and is also part of the path
                                    exit_id += 1 # go on, this is not the exit you are looking for
                                else: # neighbour is an exit and part of the path
                                    break # no need to look further
        return exit_id
        
        
# MapData class to keep together all static data related to the map
class MapData():
    
    def __init__(self,scenario):
        # priority variables to keep track of the biggest and lowest priorities among edges (THIS IS NOT USEFUL, IT WILL PROBABLY BE REMOVED)
        self.maxprio = 0
        self.minprio = 100
        self.scenario = scenario # scenario name
        traci.start(['sumo','-c','osm_'+str(scenario)+'.sumocfg','--step-length','1.0'])
        self.graphdict, self.graphmap, self.net, self.connections, self.edgegraph = build_graph(scenario) # obtain data from the net.xml file
        self.destinations, self.sources = parse_file_for_nodes(str(scenario)+'ScenarioData/config.txt') # define destination and source edges of vehicles
        self.target_weights = []
        self.targets = {}
        # define targets for vehicles and their respective weights (these will be used to spawn cars in realistic proportions)
        for t in self.destinations:
            self.target_weights.append(t[2])
            self.targets[t[1]] = t[0]
        edge_list = self.net.getEdges()
        self.edgelist = []
        self.edge2target = {}
        # build list of only traversable edges, also initialise data structure for keeping track of which targets can be reached by which edges
        for e in edge_list:
            if e.allows('passenger'):
                prio = self.net.getEdge(e.getID()).getPriority()
                # additionally, update of maximum and minimum priority variables (THIS WILL PROBABLY BE REMOVED)
                if prio > self.maxprio:
                    self.maxprio = prio
                if prio < self.minprio:
                    self.minprio = prio
                self.edgelist.append(e)
                self.edge2target[e.getID()] = {}
                for t in self.targets:
                    self.edge2target[e.getID()][self.targets[t]] = (False,False,[])
        self.edge_dict = {}
        # initialise  edge_dict (this will be used by the behaviour creation module)
        for e in self.edgelist:
            self.edge_dict[e.getID()] = self.edgelist.index(e)
        self.works = parse_file_for_work(str(scenario)+'ScenarioData/works.txt') # load work areas from file
        # roundabout creation part, load roundabouts from a file or create them from scratch (creation takes a lot of time)
        self.roundabouts = None
        create_roundabouts = False
        if os.path.exists(str(scenario)+'ScenarioData/roundabouts.txt'):
            self.roundabouts = self.read_roundabouts()
        elif create_roundabouts:
            self.roundabouts = self.find_roundabouts()
        if self.roundabouts is not None: # additionally keep track of streets inside roundabouts to improve efficiency for checks
            self.streets_in_roundabouts = {}
            for r in self.roundabouts:
                for road in r.roads:
                    self.streets_in_roundabouts[road] = r.id
        traci.close()
    
    # recursive method for checking if an edge belongs in a roundabout while creating them, inputs are:
    # - instance of MapData,
    # - edge whose belonging to a roundabout is being checked,
    # - one of the edges neighbouring the checked one (or a close one), used for looking for belonging to a roundabout,
    # - metric length of edges between the checked edge and the neighbouring one
    def check_roundabouts(self,calling_edge,current_edge,len_up_to_now):
        net = self.net
        start_length = net.getEdge(current_edge).getLength()
        if start_length+len_up_to_now>75 or net.getEdge(calling_edge).getPriority()!=net.getEdge(current_edge).getPriority():
            return None # roundabouts are usually no longer than 75 meters, so if current_edge is farther than 75 meters from calling_edge or priorities are different (SUMO gives roads in the same roundabout the same priority), go back in the recursion, this is no good road
        o_outs = net.getEdge(current_edge).getOutgoing()
        outs = []
        ret_val = None
        for e in o_outs: # check neighbouring edges
            if net.getEdge(e.getID()).allows('passenger'):
                outs.append(e)
        for edge in outs: # check neighbouring edges
            if edge.getID() == calling_edge and net.getEdge(calling_edge).getPriority()==net.getEdge(edge.getID()).getPriority(): # the neighbour is the original edge, roundabout is found
                return [current_edge]
            ret_val = self.check_roundabouts(calling_edge,edge.getID(),start_length+len_up_to_now) # look for roundabout among the neighbours of the neighbour
            if ret_val is not None: # a roundabout was found, go back in the recursion and extend the list of cars in the roundabout
                ret_new = [current_edge]
                ret_new.extend(ret_val)
                return ret_new
        return ret_val
    
    # method for identifying roundabouts in the map, inputs are:
    # - instance of MapData
    def find_roundabouts(self):
        roundabouts = []
        roads_in_roundabouts = []
        rs_index = 0
        for edge in self.edgelist: # for every edge among the traversable ones and that is not already inside a roundabout
            if edge.getID() not in roads_in_roundabouts:
                ret_value = self.check_roundabouts(edge.getID(),edge.getID(),0) # look for a roundabout
                if ret_value is not None: # if roundabout is found, save it
                    roads_in_roundabouts.extend(ret_value)
                    r = Roundabout('roundabout'+str(rs_index),ret_value,mapdata=self)
                    roundabouts.append(r)
                    self.write_roundabout(r) # and write roundabout on file
                    rs_index += 1
        return roundabouts
    
    # method for writing roundabout data on a file, inputs are:
    # - instance of MapData,
    # - instance of Roundabout to write
    def write_roundabout(self,r):
        f = open('roundabouts_Unisa.txt','a')
        # in order, write on file
        f.write(str(len(r.roads))+';') # the number of roads inside the roundabout
        for road in r.roads: # the list of roads belonging to the roundabout
            f.write(road+';')
        f.write(str(len(r.exits))) # the number of exits of the roundabout
        for ex in r.exits: # the list of exits of the roundabout
            f.write(';'+str(ex))
        f.write('\n')
        f.close()

    # method for loading Roundabout instances starting from a file, inputs are:
    # - instance of MapData
    def read_roundabouts(self):
        f = open(str(self.scenario)+'ScenarioData/roundabouts.txt','r')
        content = f.readlines()
        f.close()
        roundabouts = {}
        roundabout_list = []
        index = 0
        for r in content: # for every line of the file
            rid = 'r'+str(index)
            roundabouts[rid] = {}
            roundabouts[rid]['roads'] = []
            roundabouts[rid]['exits'] = []
            split_content = r.split(';')
            r_len = int(split_content[0])
            for i in range(r_len):
                roundabouts[rid]['roads'].append(split_content[1+i].replace('\n',''))
            e_len = int(split_content[1+r_len])
            for i in range(e_len):
                roundabouts[rid]['exits'].append(split_content[2+r_len+i].replace('\n',''))
            rb = Roundabout(rid,roundabouts[rid]['roads'],exits=roundabouts[rid]['exits']) # create Roundabout instance with its roads and exits
            roundabout_list.append(rb)
            index += 1
        return roundabout_list
        
        
    