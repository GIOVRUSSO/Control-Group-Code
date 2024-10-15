# Module for all color-related methods

import numpy as np
import traci

# method for defining car colors according to their destination, inputs are:
# - string defining the car destination
def car_color(type):
    if type=='NAPOLI':
        return 255,0,255,1 # fucsia
    elif type=='AVELLINO':
        return 0,255,0,1 # green
    elif type=='SALERNO':
        return 0,255,255,1 # azure
    elif type=='BENEVENTO':
        return 255,255,0,1 # yellow
    elif type=='FISCIANO':
        return 255,0,0,1 # red
    elif type=='PENTA':
        return 0,0,255,1 # blue
    return 255,255,255,1

# method for defining street colors according to their algorithm, inputs are: 
# - string defining the algorithm
def alg_color(type):
    if type=='e_bfs':
        return 255,0,255,1 # fucsia
    elif type=='e_dijkstra':
        return 0,255,0,1 # green
    elif type=='e_greedybfs':
        return 0,255,255,1 # azure
    elif type=='e_astar':
        return 255,255,0,1 # yellow
    return 255,255,255,1

# method for turning RBG array of colors into integer value, inputs are:
# - an array containing the RBG values of the color to be turned into integer
def getIfromRGB(rgb):
    red = rgb[0]
    green = rgb[1]
    blue = rgb[2]
    RGBint = (red<<16) + (green<<8) + blue
    return RGBint

# method for coloring edges in the scenario according to their reward (MUST BE ADAPTED TO EACH COST FUNCTION), inputs are:
# - reward array,
# - data structure containing all static data related to the map,
# - data structure keeping track of previously colored streets
def colorMap(r,mapdata,rewards4colors):
    edgelist = mapdata.edgelist
    maxrew = np.max(r)
    maxre4=0
    minrew = np.min(r)
    minre4=0
    vals = list(rewards4colors.values())
    sumvals = np.sum(vals)
    if sumvals!=0:
        maxre4 = np.max(vals)
        minre4 = np.min(vals)
    maxr = 0
    minr = 0
    maxr = maxrew
    minr = minrew
    if sumvals == 0:
        maxr = 1000
    print('maxr '+str(maxr)+' minr '+str(minr)+' maxre4 '+str(maxre4)+' minre4 '+str(minre4)+' maxrew '+str(maxrew)+' minrew '+str(minrew))
    for edgeindex in range(len(edgelist)):
        ed = edgelist[edgeindex].getID()
        rvalue = minr
        colorvalue = 0
        if r[edgeindex]!=0:
            rewards4colors[ed] = r[edgeindex]
            rvalue = rewards4colors[ed]
            colorvalue = int(abs(rvalue-minr)/abs(maxr-minr)*1000)+10
        traci.edge.setParameter(ed,'color',colorvalue)