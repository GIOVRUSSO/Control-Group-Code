import xml.etree.ElementTree as ET

import numpy as np


class Agent():
    def __init__(self, scenario, add_path, edge_start):
        self.scenario = scenario
        self.start = {'edge': edge_start,
                      'index': self.scenario.edge_list.index(self.scenario.net.getEdge(edge_start))}
        self.park_list = {'id_park': [],
                          'edge': [],
                          'index': [],
                          'tot_slots': [],
                          'free_slots': []}

        self.state = self.start['index']
        self.route = [self.start['edge']]

        # suppose that in the sumo additional file the car parks of interest are already present
        tree_add = ET.parse(add_path)
        additional = tree_add.getroot()
        for e in additional:
            if e.tag == 'parkingArea':
                self.park_list['id_park'].append(e.attrib['id'])
                self.park_list['edge'].append(e.attrib['lane'][:-2])
                self.park_list['index'].append(
                    self.scenario.edge_list.index(self.scenario.net.getEdge(e.attrib['lane'][:-2])))
                self.park_list['tot_slots'].append(int(e.attrib['roadsideCapacity']))
                self.park_list['free_slots'].append(int(e.attrib['roadsideCapacity']))

        self.n_destinations = len(self.park_list['id_park'])
        self.PMF_targets = np.zeros(shape=(self.n_destinations, self.scenario.n_edge, self.scenario.n_edge),
                                    dtype=float)
        self.total_cases_targets = np.zeros(shape=(self.n_destinations, self.scenario.n_edge), dtype=int)
        self.rewards = -50 * np.ones(shape=(self.n_destinations, self.scenario.n_edge), dtype=float)
        self.__update_reward()

    def build_PMF_targets(self, path_route_target):
        # the construction of the target PMFs was done manually
        for i in range(self.n_destinations):
            tree_routes = ET.parse(path_route_target + 'target_' + str(i) + '_man.rou.xml')
            routes = tree_routes.getroot()
            # print("Update PMF target", i)
            self.PMF_targets[i], self.total_cases_targets[i] = self.scenario.get_PMF_clean()
            for vehicle in routes:
                edges = vehicle[0].attrib['edges'].split(" ")
                self.scenario.update_PMF(self.PMF_targets[i], self.total_cases_targets[i], "target_" + str(i), edges)

    def has_changed_slots(self, parkingarea):
        for i in range(self.n_destinations):
            id_park = self.park_list['id_park'][i]
            free = self.park_list['tot_slots'][i] - parkingarea.getVehicleCount(id_park)
            # print("parking", id_park, 'slot free', free)
            if free != self.park_list['free_slots'][i]:
                return True
        return False

    def update_slots(self, parkingarea):
        count = []
        for i in range(self.n_destinations):
            id_park = self.park_list['id_park'][i]
            count.append(self.park_list['tot_slots'][i] - parkingarea.getVehicleCount(id_park))
        self.park_list['free_slots'] = count
        self.__update_reward()

    def __update_reward(self):
        for i in range(self.n_destinations):
            id_park = self.park_list['id_park'][i]
            index_park = self.park_list['index'][i]
            slot_free = self.park_list['free_slots'][i]
            if slot_free == 0:
                self.rewards[i][index_park] = -10000
            else:
                self.rewards[i][index_park] = slot_free

    def convert_route(self, states_list):
        route = []
        for s in states_list:
            route.append(self.scenario.edge_list[s].getID())
        self.route = route

    def get_route(self):
        return self.route

    def set_state(self, edge):
        self.state = self.scenario.edge_list.index(self.scenario.net.getEdge(edge))
