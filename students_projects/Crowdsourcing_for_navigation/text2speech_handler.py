# Module for playing direction and warning audios

from gtts import gTTS
from tempfile import TemporaryFile
import pygame
from behaviours_maker import valid_neighbors

# method for converting a SUMO direction into a string in both english and italian language, inputs are:
# - string representing the direction,
# - string representing the language in which the direction must be translated
def convertDir2String(dir,lang):
    if dir=='s':
        return 'straight' if lang=='en' else 'dritto'
    if dir=='l':
        return 'left' if lang=='en' else 'a sinistra'
    if dir=='r':
        return 'right' if lang=='en' else 'a destra'
    if dir=='R':
        return 'right' if lang=='en' else 'a destra'
    if dir=='L':
        return 'left' if lang=='en' else 'a sinistra'
    if dir=='t':
        return 'back' if lang=='en' else 'indietro'

# method for approximating a value to its closest multiple of 10, inputs are:
# - the value to approximate    
def approx10(value):
    v = int(value/10)
    return v*10

# method for approximating a value to its closest multiple of 1000, inputs are:
# - the value to approximate    
def approx1000(value):
    v = int(value/1000)
    return v*1000

# method for playing direction audio, inputs are:
# - data structure containing all static data related to the map,
# - edge from which the audio is being asked to play,
# - edge at the crossing of interest because of which the audio is being played,
# - a string representing the required language to use,
# - a distance value (optional),
# - an edge to take into account while looking for directions to the edge at the crossing of interest (optional),
# - a flag expressing whether the audio is related to an edge inside a roundabout or not (optional),
# - a path to take into account while looking for directions in roundabouts (optional)
def playAudio(mapdata,current,prox,lan,dist=None,prox_edge=None,roundabout=False,path=None):
    net = mapdata.net
    connections = mapdata.connections
    roadname = net.getEdge(prox.getID()).getName()
    c_roadname = net.getEdge(current).getName()
    roadlen = approx10(int(net.getEdge(current).getLength())) if dist is None else approx10(dist) # approximate distance
    km = False
    if roadlen>=1000: # if dealing with kilometers, make a sentence with kilometers instead of meters
        roadlen = int(approx1000(int(roadlen))/1000)
        km = True
    keepGoing = roadname==c_roadname # check if vehicle needs to keep going on their street
    lentext = ("In "+str(roadlen)+(' kilo' if km else ' ')+"meter"+("s" if not km or roadlen>1 else "")+", " if lan=='en' else "Tra "+str(roadlen)+(' chilo' if km else ' ')+"metr"+("i" if not km or roadlen>1 else "o")+", ") if (roadlen>=20 and not km) or km else "" # only declare the distance if it is bigger than 20 meters
    aud_text = lentext
    play = True
    if not roundabout: # edge of the crossing of interest is not in a roundabout
        aud_text = aud_text+("go " if lan=='en' else "procedi ")
        direction = None
        if prox_edge is None: # take into account current and prox edges for directions
            if prox.getID() in connections[current]:
                direction = connections[current][prox.getID()]
        else: # take into account prox_edge and prox edges for directions
            if prox.getID() in connections[prox_edge.getID()]:
                direction = connections[prox_edge.getID()][prox.getID()]
        if direction is not None: # direction was found
            directionstring = convertDir2String(direction,lan)
            aud_text += directionstring
            if len(roadname)>2: # create string to turn into audio
                aud_text += ((' towards ' if not keepGoing else ' along ') if lan=='en' else (' verso ' if not keepGoing else ' lungo '))+str(roadname)
        else: # no direction was found, no need to play audio
            play = False
    else: # edge of the crossing of interest is in a roundabout
        if prox_edge is None: # audio is required inside a roundabout
            if prox.getID() not in mapdata.streets_in_roundabouts: # the other edge is a roundabout exit
                aud_text = ("exit the roundabout." if lan=='en' else "esci dalla rotonda.")
            else: # not a roundabout exit, no audio is required
                play = False
        else: # audio is required outside of a roundabout
            aud_text = aud_text+("in the roundabout, take the " if lan=='en' else "alla rotonda, prendi la ")
            val = mapdata.roundabouts[int(mapdata.streets_in_roundabouts[prox_edge.getID()].replace('r',''))].exit_order(mapdata,path) # find roundabout exit to look for
            aud_text = aud_text+order_number_string(val,lan)+(" exit." if lan=='en' else " uscita.")
    if play: # play audio using pygame library
        aud = gTTS(text=aud_text,lang=lan,slow=False)
        f = TemporaryFile()
        aud.write_to_fp(f)
        f.seek(0)
        pygame.mixer.init()
        pygame.mixer.music.load(f)
        pygame.mixer.music.play()

# method for playing audios warning about work-in-progress areas, inputs are:
# - data structure containing all static data related to the map,
# - the road in the wip area,
# - sequence of roads that lead to the wip area,
# - string representing the audio required language
def playWarningAudio(mapdata,road,roadsequence,lan):
    net = mapdata.net
    roadname = net.getEdge(road).getName()
    nonameroad = len(roadname)<=2
    if nonameroad: # look for a name of a road close to the wip area
        for r in roadsequence:
            rname = net.getEdge(r).getName()
            if len(rname)>2:
                roadname = rname
                break
    aud_text = 'Warning, work in progress ' if lan=='en' else 'Attenzione, lavori in corso '
    if nonameroad and len(roadname)>2: # use name of a nearby road
        aud_text += 'close to '+str(roadname) if lan=='en' else 'vicino a '+str(roadname)
    elif not nonameroad: # use wip area road name
        aud_text += 'at '+str(roadname) if lan=='en' else 'a '+str(roadname)
    # play audio using pygame library
    aud = gTTS(text=aud_text,lang=lan,slow=False)
    f = TemporaryFile()
    aud.write_to_fp(f)
    f.seek(0)
    pygame.mixer.init()
    pygame.mixer.music.load(f)
    pygame.mixer.music.play()

# method for turning numbers into cardinal order strings, inputs are:
# - integer value to turn into string,
# - a string representing the required audio language
def order_number_string(value,lan):
    if value==1:
        return 'first' if lan=='en' else 'prima'
    if value==2:
        return 'second' if lan=='en' else 'seconda'
    if value==3:
        return 'third' if lan=='en' else 'terza'
    if value==4:
        return 'fourth' if lan=='en' else 'quarta'
    if value==5:
        return 'fifth' if lan=='en' else 'quinta'

# method to look for next crossing, inputs are:
# - data structure containing all static data related to the map,
# - a reference path
def search_next(mapdata,path):
    net = mapdata.net
    warning_edge = None
    prev_warning_edge = None
    prevlen = 0
    for e in path: # loop in the path until multiple choices are available at a crossing
        neighbours = valid_neighbors(net.getEdge(e),mapdata)
        if prevlen > 1:
            warning_edge = e
            break
        if len(neighbours)>=2:
            prevlen = len(neighbours)
        prev_warning_edge = e
    print('warning edge is '+str(warning_edge))
    print('prev warning edge is '+str(prev_warning_edge))
    if warning_edge is not None: # if a crossing is found
        pathlen = 0
        for e in path:
            pathlen += net.getEdge(e).getLength()
            if e==prev_warning_edge:
                break
        if prev_warning_edge in mapdata.streets_in_roundabouts: # warn about distance and the fact that next up is a roundabout
            return True,True,pathlen,warning_edge,prev_warning_edge
        else: # warn about distance and the fact that next up is a crossing
            return True,False,pathlen,warning_edge,prev_warning_edge
    return False,False,0,None,None