# Script to create a visual representation of the paths created by the agents, useful for debugging

import traci
from algorithms import *
from mapdata import *
from behaviours_maker import index2path
from behaviours_maker import index2alg
from behaviours_maker import alg_color
from colors import getIfromRGB
import time
from behaviours_maker import calculate_pathlen

# decomment the starting edge from which you want the animation to start
# start = '766350967' # start edge of cars in Salerno city centre map, located in Concordia parking area
# start = '-79867013#1' # start edge of cars in Unisa map, located in Multipiano parking area
start = '-579690548#1' # start edge of cars in Unisa map, located in Campus parking area
# end = '50827474' # end edge of cars in Salerno city centre map, direction towards Avellino
# end = '-40567772#0' # end edge of cars in Unisa map, direction towards Avellino
end = '330222144' # end edge of cars in Unisa map, direction towards Napoli
# end = '93450829' # end edge of cars in Salerno city centre map, direction towards Napoli
# end = '102235300#2' # end edge of cars in Salerno city centre map, direction towards other cities in the province of Salerno
SCENARIO = 'Unisa'
dijkstrabased = {} # data structure to keep track of first edges chosen by Dijkstra algorithm for paths
mapdata = MapData(SCENARIO)
works = mapdata.works
start = mapdata.net.getEdge(start)
traci.start(['sumo-gui','-c','osm_'+str(SCENARIO)+'.sumocfg','--step-length',str(1.0)])
paths = [None]*4
for i in range(4):
    # for k in mapdata.edgelist:
    #     traci.edge.setParameter(k.getID(),'color',0)
    paths[i] = index2path(i,end,start,mapdata,works=works)
    if i == 0:
        dijkstrabased[start.getID()] = paths[i][1] # update data structure to keep track of first edges chosen by Dijkstra algorithm for paths
    colorv = getIfromRGB(list(alg_color(index2alg(i)))[0:3])        
    print(calculate_pathlen(paths[i],works,mapdata))  
    # color paths     
    for e in paths[i]:
        traci.edge.setParameter(e,'color',colorv)
        time.sleep(0.01)
        traci.simulationStep()
    time.sleep(5.0)
    traci.simulationStep()
traci.close()
    