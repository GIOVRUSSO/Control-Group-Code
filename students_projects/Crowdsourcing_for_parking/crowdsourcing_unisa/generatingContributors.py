import os
import xml.etree.ElementTree as ET

import numpy as np


class GeneratingContributors():

    def __init__(self, n_contribs, scenario):
        self.n_contribs = n_contribs
        self.scenario = scenario
        self.PMF_contribs = np.zeros(shape=(n_contribs, self.scenario.n_edge, self.scenario.n_edge), dtype=float)
        self.total_cases_contribs = np.zeros(shape=(n_contribs, self.scenario.n_edge), dtype=int)

    def build_PMF_contribs(self, n_tests):
        for i in range(self.n_contribs):
            self.PMF_contribs[i], self.total_cases_contribs[i] = self.scenario.get_PMF_clean()
        # random route generation
        period = 1
        for j in range(n_tests):
            print("Test", j)
            # the construction of the routes is done in such a way that each contributors cross an edge where there is a parking space
            randomTripCommand = "randomTrips --random --end " + str(
                self.n_contribs * period) + " -n sumo_files/osm.net.xml --period " + str(
                period) + " --validate --remove-loops -r sumo_files/routes_contribs.rou.xml " \
                          "--intermediate 1 --weights-prefix=sumo_files/weights_contribs"
            os.system(randomTripCommand)

            tree_routes = ET.parse('sumo_files/routes_contribs.rou.xml')
            routes = tree_routes.getroot()
            for i in range(len(routes)):
                edges = routes[i][0].attrib['edges'].split(" ")
                print("Update PMF contribs", i)
                self.scenario.update_PMF(self.PMF_contribs[i], self.total_cases_contribs[i],
                                         "contrib_" + str(i), edges)

# #route control
# tree_routes = ET.parse('../sumo_files/routes_contribs.rou.xml')
# routes = tree_routes.getroot()
# for i in range(self.n_contribs):
#     if routes[i].attrib['id'] != str(i):
#         routes.insert(i, copy.deepcopy(routes[i-1]))
#         vehicle = routes[i]
#         vehicle.attrib['id'] = str(i)
#         vehicle.attrib['depart'] = str(i)+'.00'
# tree_routes.write('../sumo_files/routes.rou.xml')


# #redirection in parking lots
# parkings = []
# tree_add = ET.parse('../sumo_files/add.add.xml')
# additional = tree_add.getroot()
# for child in additional:
#     if child.tag == 'parkingArea':
#         parkings.append((child.attrib['id'], child.attrib['lane'].split('_')[0]))
#
# tree_routes = ET.parse('../sumo_files/routes.rou.xml')
# routes = tree_routes.getroot()
# for vehicle in routes:
#     edges = vehicle[0].attrib['edges']
#     for parking in parkings:
#         if parking[1] in edges:
#             stop = ET.SubElement(vehicle, 'stop')
#             stop.attrib['parkingArea'] = parking[0]
#             stop.attrib['duration'] = str(random.randint(200,400))
# tree_routes.write('../sumo_files/routes.rou.xml')
