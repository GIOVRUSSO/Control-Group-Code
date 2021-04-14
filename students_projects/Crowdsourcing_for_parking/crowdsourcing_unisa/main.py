import copy
import os
import pickle
import sys

import sumolib
import traci

from agent import Agent
from crowdsourcing import Crowdsourcing
from generatingContributors import GeneratingContributors
from scenario import Scenario

# config parameter

n_contributors = 100
sumo_net_path = "sumo_files/osm.net.xml"
sumo_add_path = "sumo_files/add.add.xml"
sumo_routes_sim_path = "sumo_files/routes.rou.xml"
route_target_path = "sumo_files/target/"
edge_start = "62166872"
n_test_contribs = 1
update_interval = 5
scena = "32"


def change_route_color(route):
    for edge in route:
        traci.gui.toggleSelection(objID=edge, objType="edge")
    return


if __name__ == '__main__':
    scenario = Scenario(sumo_net_path)

    generator = GeneratingContributors(n_contributors, scenario)
    generator.build_PMF_contribs(n_test_contribs)

    agent_0 = Agent(scenario, sumo_add_path, edge_start)
    agent_0.build_PMF_targets(route_target_path)
    n_agents, n_foes, period, step_agent_start, lay_off, name_scenario = \
        scenario.set_scenario(scena, edge_start, agent_0.park_list, sumo_routes_sim_path)

    agents = []  # stuttura che mantiene le istanze degli agenti
    crowds = []  # struttura che mantiene le istanze alla classe crowdsoucing degli agenti

    for i in range(n_agents):
        agents.append(copy.copy(agent_0))
        crowds.append(Crowdsourcing(generator, agents[i]))

    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")

    sumoBinary = sumolib.checkBinary('sumo-gui')
    sumoCmd = [sumoBinary, "-c", r"sumo_files/osm.sumocfg"]
    traci.start(sumoCmd)
    print("Starting SUMO...")

    # initial route for the agents
    for i in range(n_agents):
        traci.route.add("route_agent." + str(i), agents[i].get_route())
        traci.vehicle.add("agent." + str(i), "route_agent." + str(i), depart=str(step_agent_start + i * period))
        traci.vehicle.setColor("agent." + str(i), color=(255, 0, 0, 255))

    step = 0

    # structure for saving the data for plot
    step_agents = [[] for i in range(n_agents)]
    distance_agents = [[] for i in range(n_agents)]
    step_foes = [[] for i in range(n_foes)]
    distance_foes = [[] for i in range(n_foes)]
    stop_agent = [False for i in range(n_agents)]
    stop_foe = [False for i in range(n_foes)]
    vehicle_biblio = []
    vehicle_terminal = []
    vehicle_multipiano = []

    # structure for saving average and variance
    means_agent = []
    variance_agent = []
    means_foe = []
    variance_foe = []

    # strat simulation
    while traci.simulation.getMinExpectedNumber() > 0:
        print('new step', step)
        vehicle_biblio.append(traci.parkingarea.getVehicleCount("biblioteca"))
        vehicle_terminal.append(traci.parkingarea.getVehicleCount("terminal"))
        vehicle_multipiano.append(traci.parkingarea.getVehicleCount("multipiano"))
        last_dist_agent = []
        last_dist_foe = []
        # code for control the agent
        for i in range(n_agents):
            # print("Agent: ", i)
            if "agent." + str(i) in traci.vehicle.getIDList():
                # traci.gui.trackVehicle(viewID=traci.gui.DEFAULT_VIEW, vehID="agent."+str(i))
                current_edge = traci.vehicle.getRoadID('agent.' + str(i))
                distance = traci.vehicle.getDistance("agent." + str(i))
                # code for saving data for the plot
                # -pow(2, 30) is error value for .getDistance, in this case the vehicle is stationary:
                if distance != -pow(2, 30) and stop_agent[i] == False:
                    step_agents[i].append(step)
                    distance_agents[i].append(distance)
                else:
                    stop_agent[i] = True
                # boolean that control if the parking area has changed slots free
                choice = agents[i].has_changed_slots(traci.parkingarea)
                # conditions for start the algorithm: 1 or 2
                # 1 is for when the agent starts
                # 2 is for (A and B and C and D)
                # A when the agent not is arrived at the destination
                # B the number of slots free are changed
                # C in order not to slow down the simulation, the update of the routes is sampled according to update_interval
                # D ":" is the first character of the value of the traci.vehicle.getRoadID() when the vehicle is on an
                # intersection, in this case we have no corresponding status
                if distance == 0.0 or (current_edge not in agents[i].park_list["edge"] and choice
                                       and step % update_interval == 0 and current_edge[0] != ":"):
                    # print('start algorithm')
                    agents[i].update_slots(traci.parkingarea)
                    agents[i].set_state(current_edge)
                    old_route = agents[i].get_route()
                    crowds[i].step_route()
                    route = agents[i].get_route()
                    # code for visualize the agent's route on the simulation
                    if n_agents == 1:
                        change_route_color(old_route)
                        change_route_color(route)
                    # print("new route is", route)
                    traci.vehicle.setRoute('agent.' + str(i), route)
                    index = agents[i].park_list['edge'].index(route[-1])
                    traci.vehicle.setParkingAreaStop('agent.' + str(i), agents[i].park_list['id_park'][index],
                                                     duration=lay_off)
        # code for save the data related at the sumo car
        for j in range(n_foes):
            if "foe." + str(j) in traci.vehicle.getIDList():
                distance_foe = traci.vehicle.getDistance("foe." + str(j))
                # print("distance foe", distance_foe)

                if distance_foe != -pow(2, 30) and stop_foe[j] == False:
                    step_foes[j].append(step)
                    distance_foes[j].append(distance_foe)
                else:
                    stop_foe[j] = True

        # code for saving average and variance
        # for i in range(len(distance_agents)):
        #     if len(distance_agents[i])>0 and step_agents[i][-1] == step:
        #         last_dist_agent.append(distance_agents[i][-1])
        # means_agent.append(np.mean(last_dist_agent))
        # variance_agent.append(np.var(last_dist_agent))
        #
        # for i in range(len(distance_foes)):
        #     if len(distance_foes[i])>0 and step_foes[i][-1] == step:
        #         last_dist_foe.append(distance_foes[i][-1])
        # means_foe.append(np.mean(last_dist_foe))
        # variance_foe.append(np.var(last_dist_foe))

        traci.simulationStep()
        step += 1
    traci.close()

    pickle.dump(
        (name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal,
         vehicle_multipiano, means_agent, variance_agent, means_foe, variance_foe),
        open("output_data/scenario_" + scena + ".p", "wb"))
