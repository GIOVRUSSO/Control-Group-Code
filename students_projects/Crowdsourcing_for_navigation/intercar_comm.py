# Module for gathering methods of cars signalling wip areas

import traci
from behaviours_maker import valid_neighbors
from queue import PriorityQueue
from text2speech_handler import playWarningAudio

PLAY_AUDIO = False # flag for playing audio as soon as a wip area is detected
workAudioPlayed = [] # data structure to keep track of audios played for the different wip areas

# method for looking for alternative roads as soon as a road is considered "wip", inputs are:
# - road containing a "wip" area,
# - target edge of the vehicle signalling the wip area,
# - data structure containing all static data related to the map
def find_alternative(road,target,mapdata):
    net = mapdata.net
    edgegraph = mapdata.edgegraph
    explore = PriorityQueue()
    explore.put(road)
    returnroads = []
    while not explore.empty():
        edge = net.getEdge(explore.get())
        fromnode_edges = edge.getFromNode().getIncoming() # look for incoming edges of the edge
        for inced in fromnode_edges:
            edid = inced.getID()
            if net.getEdge(edid).allows('passenger'): # if edge is traversable
                if edge.getID() in edgegraph[edid] and edid not in returnroads: # check that edge's neighbours
                    vn = valid_neighbors(inced,mapdata,target)
                    if len(vn)==1: # it has no neighbours other than the wip area (or roads preceding it with no alternative)
                        returnroads.append(edid)
                        explore.put(edid) # keep exploring
    return returnroads
    
# method for signalling a wip area, inputs are:
# - list of real wip areas,
# - road from which the vehicle is trying to signal the wip area,
# - list of vehicles,
# - target edge of the vehicle trying to signal the wip area,
# - data structure containing all static data related to the map,
# - flag identifying whether the calling vehicle is an agent or not,
# - instance of VehicleData related to the calling vehicle,
# - language (useful for deciding warning audio language),
# - value expressing the threshold over which the wip area is considered truly one
def tweet(works,road,vehs,end_edge,mapdata,agent,vehicle,LANG,THRESHOLD):
    prevval = works[road]
    works[road] += (10 if agent else 2)*vehicle.influence # update signal value for wip area, trust agents more than non-agents
    if works[road]>=THRESHOLD and prevval<THRESHOLD: # wip signal value surpassed threshold
        for v in vehs:
            if v.__contains__('agent'):
                roadsequence = find_alternative(road,end_edge[v],mapdata) # look for alternative, until then warn about roads preceding the wip area
                for r in roadsequence:
                    vehs[v].weightened[r] = len(roadsequence)
                if PLAY_AUDIO and road not in workAudioPlayed:
                    playWarningAudio(mapdata,road,roadsequence,LANG)
                    workAudioPlayed.append(road)
        return True # wip area has been fully signalled
    return False # wip area has yet to be considered one