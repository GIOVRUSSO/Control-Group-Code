# Module containing methods for parsing specific files

# method for loading data about target edges and source edges for vehicles, inputs are:
# - file name
def parse_file_for_nodes(filename):
    dataread = []
    targets = []
    sources = []
    with open(filename,'r') as rf:
        dataread = rf.readlines()
    for el in dataread:
        el = el.replace('\n','')
        pieces = el.split(':')
        if pieces[0]=='target':
            targets.append((pieces[1],pieces[2],float(pieces[3])))
        elif pieces[0]=='start':
            sources.append(pieces[1])
    return targets,sources

# method for loading edge ids of work-in-progress areas, inputs are:
# - file name
def parse_file_for_work(filename):
    works = {}
    dataread = []
    with open(filename,'r') as rf:
        dataread = rf.readlines()
    for el in dataread:
        el = el.replace('\n','')
        works[el] = 0
    return works

# method for loading data related to checkpoints (THIS WILL PROBABLY BE REMOVED AS IT IS UNNECESSARY), inputs are:
# - file name
def parse_file_for_checkpoints(filename):
    dataread = []
    targets = []
    with open(filename,'r') as rf:
        dataread = rf.readlines()
    for el in dataread:
        el = el.replace('\n','')
        pieces = el.split(':')
        if pieces[0]=='target':
            targets.append((pieces[1],pieces[2],float(pieces[3])))
    checkpoints = {}
    for t in targets:
        if t[0] not in checkpoints:
            checkpoints[t[0]] = {}
            checkpoints[t[0]]['speed'] = []
            checkpoints[t[0]]['flow'] = 0
        if 'place' not in checkpoints[t[0]]:
            checkpoints[t[0]]['place'] = []
        checkpoints[t[0]]['place'].append(t[1])
    print(checkpoints)
    return checkpoints