import xml.etree.ElementTree as ET

import numpy as np
import sumolib


class Scenario():
    def __init__(self, net_path):
        self.net = sumolib.net.readNet(net_path)
        self.edge_list = self.net.getEdges()
        self.n_edge = len(self.edge_list)
        self.PMF_adjacency = np.zeros(shape=(self.n_edge, self.n_edge), dtype=float)
        self.total_cases = np.zeros(shape=self.n_edge, dtype=int)

        for i in range(self.n_edge):
            edges_outgoing = self.edge_list[i].getOutgoing()
            for e in edges_outgoing:
                j = self.edge_list.index(e)
                self.PMF_adjacency[i][j] = 1 / len(edges_outgoing)
            self.total_cases[i] = len(edges_outgoing)
        np.savetxt(fname='PMF_adjancency.csv', fmt='%.5f', X=self.PMF_adjacency, delimiter=',', newline='\n')
        np.savetxt(fname='total_cases.csv', fmt='%d', X=self.total_cases, delimiter=',', newline='\n')

        self.switcher = {
            "00": self.scenario_00,
            "01": self.scenario_01,
            "02": self.scenario_02,
            "11": self.scenario_11,
            "12": self.scenario_12,
            "21": self.scenario_21,
            "22": self.scenario_22,
            "23": self.scenario_23,
            "31": self.scenario_31}

    def get_PMF_clean(self):
        return self.PMF_adjacency.copy(), self.total_cases.copy()

    def update_PMF(self, PMF, total_cases, save_file, route):
        for i in range(len(route) - 1):
            edge1 = self.net.getEdge(route[i])
            index1 = self.edge_list.index(edge1)
            edge2 = self.net.getEdge(route[i + 1])
            index2 = self.edge_list.index(edge2)
            case_index1 = total_cases[index1]
            for j in range(self.n_edge):
                if PMF[index1][j] != 0:
                    if j == index2:
                        PMF[index1][j] = ((PMF[index1][j] * case_index1) + 1) / (case_index1 + 1)
                    else:
                        PMF[index1][j] = (PMF[index1][j] * case_index1) / (case_index1 + 1)
            total_cases[index1] += 1
        np.savetxt(fname="PMFs/PMF_" + save_file + ".csv", fmt='%.5f', X=PMF, delimiter=',', newline='\n')
        np.savetxt(fname="total_cases/total_cases_" + save_file + ".csv", fmt='%d', X=total_cases, delimiter=',',
                   newline='\n')

    def set_scenario(self, scena, edge_start, parking, routes_file):
        return self.switcher.get(scena, self.default)(scena, edge_start, parking, routes_file)

    def scenario_00(self, scena, edge_start, parking, routes_file):
        n_agent = 1
        n_foe = 1
        period = 0
        step_start = 0
        lay_off = 1000.0
        routes = ET.Element("routes")
        park = 0
        trip = ET.SubElement(routes, "trip")
        trip.attrib["id"] = "foe.0"
        trip.attrib["depart"] = str(step_start)
        trip.attrib["from"] = edge_start
        trip.attrib["to"] = parking['edge'][park]
        trip.attrib["color"] = "green"
        stop = ET.SubElement(trip, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][park]
        stop.attrib['duration'] = str(lay_off)
        tree = ET.ElementTree(routes)
        tree.write(routes_file)
        return n_agent, n_foe, period, step_start, lay_off, "Empty parking lot scenario"

    def scenario_01(self, scena, edge_start, parking, routes_file):
        n_agent = 1
        n_foe = 1
        period = 0
        vehicles = 50
        period_vehicle = 5
        lay_off = 1000.0
        step_start = 310

        park = 0
        routes = ET.Element("routes")
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_0"
        flow.attrib["begin"] = "0.00"
        flow.attrib["from"] = "-906517535"
        flow.attrib["to"] = parking['edge'][park]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][park]
        stop.attrib['duration'] = str(lay_off)

        trip = ET.SubElement(routes, "trip")
        trip.attrib["id"] = "foe.0"
        trip.attrib["depart"] = str(step_start)
        trip.attrib["from"] = edge_start
        trip.attrib["to"] = parking['edge'][park]
        trip.attrib["color"] = "green"
        stop = ET.SubElement(trip, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][park]
        stop.attrib['duration'] = str(lay_off)
        tree = ET.ElementTree(routes)
        tree.write(routes_file)
        return n_agent, n_foe, period, step_start, lay_off, "Biblioteca parking full, others empty"

    def scenario_02(self, scena, edge_start, parking, routes_file):
        n_agent = 1
        n_foe = 1
        period = 0
        vehicles = 50
        period_vehicle = 5
        lay_off = 1000.0
        step_start = 360

        routes = ET.Element("routes")

        terminal = 1
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_0"
        flow.attrib["begin"] = "0.00"
        flow.attrib["from"] = "-79869125#1"
        flow.attrib["to"] = parking['edge'][terminal]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][terminal]
        stop.attrib['duration'] = str(lay_off)

        biblio = 0
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_1"
        flow.attrib["begin"] = "0.00"
        flow.attrib["from"] = "-906517535"
        flow.attrib["to"] = parking['edge'][biblio]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off)

        trip = ET.SubElement(routes, "trip")
        trip.attrib["id"] = "foe.0"
        trip.attrib["depart"] = str(step_start)
        trip.attrib["from"] = edge_start
        trip.attrib["to"] = parking['edge'][biblio]
        trip.attrib["color"] = "green"
        stop = ET.SubElement(trip, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off)
        tree = ET.ElementTree(routes)
        tree.write(routes_file)
        return n_agent, n_foe, period, step_start, lay_off, "Biblioteca and Terminal full, Multipiano empty"

    def scenario_11(self, scena, edge_start, parking, routes_file):
        n_agent = 1
        n_foe = 1
        period = 0
        vehicles = 50
        period_vehicle = 5
        lay_off = 1000.0
        step_start = 275
        park = 0
        routes = ET.Element("routes")
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_11"
        flow.attrib["begin"] = "0.00"
        flow.attrib["from"] = "-906517535"
        flow.attrib["to"] = parking['edge'][park]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][park]
        stop.attrib['duration'] = str(lay_off)

        trip = ET.SubElement(routes, "trip")
        trip.attrib["id"] = "foe.0"
        trip.attrib["depart"] = str(step_start)
        trip.attrib["from"] = edge_start
        trip.attrib["to"] = parking['edge'][park]
        trip.attrib["color"] = "green"
        stop = ET.SubElement(trip, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][park]
        stop.attrib['duration'] = str(lay_off)
        tree = ET.ElementTree(routes)
        tree.write(routes_file)
        return n_agent, n_foe, period, step_start, lay_off, "All empty at the start, Biblioteca fills up"

    def scenario_12(self, scena, edge_start, parking, routes_file):
        n_agent = 1
        n_foe = 1
        period = 0
        vehicles = 50
        period_vehicle = 5
        lay_off = 1000.0
        step_start = 290
        biblio = 0
        routes = ET.Element("routes")

        terminal = 1
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_1"
        flow.attrib["begin"] = "0.00"
        flow.attrib["from"] = "-79869125#1"
        flow.attrib["to"] = parking['edge'][terminal]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][terminal]
        stop.attrib['duration'] = str(lay_off)

        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_0"
        flow.attrib["begin"] = "5.00"
        flow.attrib["from"] = "-906517535"
        flow.attrib["to"] = parking['edge'][biblio]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off)

        trip = ET.SubElement(routes, "trip")
        trip.attrib["id"] = "foe.0"
        trip.attrib["depart"] = str(step_start)
        trip.attrib["from"] = edge_start
        trip.attrib["to"] = parking['edge'][biblio]
        trip.attrib["color"] = "green"
        stop = ET.SubElement(trip, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off)
        tree = ET.ElementTree(routes)
        tree.write(routes_file)
        return n_agent, n_foe, period, step_start, lay_off, "All empty at the start, Biblioteca and Terminal fills up"

    def scenario_21(self, scena, edge_start, parking, routes_file):
        n_agent = 1
        n_foe = 1
        period = 0
        vehicles = 50
        period_vehicle = 5
        lay_off = 500.0
        step_start = 500
        biblio = 0
        routes = ET.Element("routes")

        multipiano = 2
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_0"
        flow.attrib["begin"] = "0.00"
        flow.attrib["from"] = "-122726137"
        flow.attrib["to"] = parking['edge'][multipiano]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][multipiano]
        stop.attrib['duration'] = str(lay_off)

        terminal = 1
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_1"
        flow.attrib["begin"] = "120.00"
        flow.attrib["from"] = "-79869125#1"
        flow.attrib["to"] = parking['edge'][terminal]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][terminal]
        stop.attrib['duration'] = str(lay_off * 3)

        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_2"
        flow.attrib["begin"] = "120.00"
        flow.attrib["from"] = "-906517535"
        flow.attrib["to"] = parking['edge'][biblio]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off * 3)

        trip = ET.SubElement(routes, "trip")
        trip.attrib["id"] = "foe.0"
        trip.attrib["depart"] = str(step_start)
        trip.attrib["from"] = edge_start
        trip.attrib["to"] = parking['edge'][biblio]
        trip.attrib["color"] = "green"
        stop = ET.SubElement(trip, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off)
        tree = ET.ElementTree(routes)
        tree.write(routes_file)
        return n_agent, n_foe, period, step_start, lay_off, "All full at the beginning, Multipiano get empty"

    def scenario_22(self, scena, edge_start, parking, routes_file):
        n_agent = 1
        n_foe = 1
        period = 0
        vehicles = 50
        period_vehicle = 5
        lay_off = 500.0
        step_start = 530
        biblio = 0
        routes = ET.Element("routes")

        multipiano = 2
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_0"
        flow.attrib["begin"] = "0.00"
        flow.attrib["from"] = "-122726137"
        flow.attrib["to"] = parking['edge'][multipiano]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][multipiano]
        stop.attrib['duration'] = str(lay_off)

        terminal = 1
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_1"
        flow.attrib["begin"] = "5.00"
        flow.attrib["from"] = "-79869125#1"
        flow.attrib["to"] = parking['edge'][terminal]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][terminal]
        stop.attrib['duration'] = str(lay_off)

        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_2"
        flow.attrib["begin"] = "130.00"
        flow.attrib["from"] = "-906517535"
        flow.attrib["to"] = parking['edge'][biblio]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off * 3)

        trip = ET.SubElement(routes, "trip")
        trip.attrib["id"] = "foe.0"
        trip.attrib["depart"] = str(step_start)
        trip.attrib["from"] = edge_start
        trip.attrib["to"] = parking['edge'][biblio]
        trip.attrib["color"] = "green"
        stop = ET.SubElement(trip, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off)
        tree = ET.ElementTree(routes)
        tree.write(routes_file)
        return n_agent, n_foe, period, step_start, lay_off, "Full at the start, Multipiano and Terminal get empty"

    def scenario_23(self, scena, edge_start, parking, routes_file):
        n_agent = 1
        n_foe = 1
        period = 0
        vehicles = 50
        period_vehicle = 5
        lay_off = 500.0
        step_start = 530
        biblio = 0
        routes = ET.Element("routes")

        multipiano = 2
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_0"
        flow.attrib["begin"] = "0.00"
        flow.attrib["from"] = "-122726137"
        flow.attrib["to"] = parking['edge'][multipiano]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][multipiano]
        stop.attrib['duration'] = str(lay_off)

        terminal = 1
        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_1"
        flow.attrib["begin"] = "5.00"
        flow.attrib["from"] = "-79869125#1"
        flow.attrib["to"] = parking['edge'][terminal]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][terminal]
        stop.attrib['duration'] = str(lay_off)

        flow = ET.SubElement(routes, "flow")
        flow.attrib["id"] = "flow_2"
        flow.attrib["begin"] = "33.00"
        flow.attrib["from"] = "-906517535"
        flow.attrib["to"] = parking['edge'][biblio]
        flow.attrib["period"] = str(period_vehicle)
        flow.attrib["number"] = str(vehicles)
        stop = ET.SubElement(flow, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off)

        trip = ET.SubElement(routes, "trip")
        trip.attrib["id"] = "foe.0"
        trip.attrib["depart"] = str(step_start)
        trip.attrib["from"] = edge_start
        trip.attrib["to"] = parking['edge'][biblio]
        trip.attrib["color"] = "green"
        stop = ET.SubElement(trip, 'stop')
        stop.attrib['parkingArea'] = parking['id_park'][biblio]
        stop.attrib['duration'] = str(lay_off)
        tree = ET.ElementTree(routes)
        tree.write(routes_file)
        return n_agent, n_foe, period, step_start, lay_off, "All full at the beginning, all parking get empty"

    def scenario_31(self, scena, edge_start, parking, routes_file):
        n_agent = 75
        n_foe = 75
        period = 10.0
        lay_off = 1000.0
        step_start = 0
        park = 0
        routes = ET.Element("routes")
        for i in range(n_foe):
            trip = ET.SubElement(routes, "trip")
            trip.attrib["id"] = "foe." + str(i)
            trip.attrib["depart"] = str(period * i)
            trip.attrib["from"] = edge_start
            trip.attrib["to"] = parking['edge'][park]
            trip.attrib["color"] = "green"
            stop = ET.SubElement(trip, 'stop')
            stop.attrib['parkingArea'] = parking['id_park'][park]
            stop.attrib['duration'] = str(lay_off)
        tree = ET.ElementTree(routes)
        tree.write(routes_file)
        return n_agent, n_foe, period, step_start, lay_off, "All empty park with 75 Crowd Cars and 75 Sumo Cars"


    def default(self, scena, edge_start, parking, routes_file):
        return self.scenario_00(scena, edge_start, parking, routes_file)
