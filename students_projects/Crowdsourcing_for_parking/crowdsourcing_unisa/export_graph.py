import pickle

import matplotlib.pyplot as plt
import numpy as np

scena = "01"

if scena == "00":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, _, _, _ = pickle.load(
        open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]

    fig, ax1 = plt.subplots()
    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance agent (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    ax1.annotate('starting cars',
                 xy=(5, 10), xycoords='data',
                 xytext=(5, 200),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='center', verticalalignment='top')

    ax1.annotate('parked cars',
                 xy=(62, 600), xycoords='data',
                 xytext=(62, 400),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='center', verticalalignment='bottom')

    ax1.plot(x_agent, y_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)

    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance foe (m)", color=color2)
    ax2.plot(x_foe, y_foe, color=color2)
    # ax2.grid()
    ax2.tick_params(axis="y", labelcolor=color2)

    fig.tight_layout()
    plt.show()
    fig.savefig("plot/scenario_" + scena + ".png", dpi=1200)

elif scena == "01":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, _, _, _ = pickle.load(
        open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    fig, ax1 = plt.subplots()
    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance agent (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    ax1.plot(x_agent, y_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)

    ax1.annotate('starting cars',
                 xy=(1200, 1), xycoords='data',
                 xytext=(1200, 350),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='left', verticalalignment='top')

    ax1.annotate('parked cars',
                 xy=(360, 660), xycoords='data',
                 xytext=(380, 660),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='left', verticalalignment='center')
    ax1.annotate(' ',
                 xy=(440, 660), xycoords='data',
                 xytext=(410, 660),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='right', verticalalignment='center')

    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance foe (m)", color=color2)
    ax2.plot(x_foe, y_foe, color=color2)
    # ax2.grid()
    ax2.tick_params(axis="y", labelcolor=color2)

    fig.tight_layout()
    plt.show()
    fig.savefig("plot/scenario_" + scena + ".png", dpi=1200)

elif scena == "02":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, _, _, _ = pickle.load(
        open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    fig, ax1 = plt.subplots()
    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance agent (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    ax1.plot(x_agent, y_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)

    ax1.annotate('starting cars',
                 xy=(400, 100), xycoords='data',
                 xytext=(400, 950),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='left', verticalalignment='top')

    ax1.annotate('parked cars',
                 xy=(570, 1700), xycoords='data',
                 xytext=(620, 1700),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='left', verticalalignment='center')
    ax1.annotate(' ',
                 xy=(740, 1700), xycoords='data',
                 xytext=(690, 1700),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='right', verticalalignment='center')

    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance foe (m)", color=color2)
    ax2.plot(x_foe, y_foe, color=color2)
    # ax2.grid()
    ax2.tick_params(axis="y", labelcolor=color2)

    fig.tight_layout()
    plt.show()
    fig.savefig("plot/scenario_" + scena + ".png", dpi=1200)

elif scena == "11":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, _, _ = pickle.load(
        open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    istant = np.argmax(vehicle_biblio)
    fig, ax1 = plt.subplots()
    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance agent (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    ax1.plot(x_agent, y_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)

    ax1.annotate('starting\ncars',
                 xy=(265, 1), xycoords='data',
                 xytext=(265, 400),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='center', verticalalignment='top')

    ax1.annotate('parked cars',
                 xy=(345, 850), xycoords='data',
                 xytext=(370, 850),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='left', verticalalignment='center')
    ax1.annotate(' ',
                 xy=(425, 850), xycoords='data',
                 xytext=(405, 850),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='right', verticalalignment='center')

    ax1.axvline(x=istant, linestyle="--")
    ax1.annotate('Biblioteca\nfills up', xy=(istant + 1, 0), xycoords='data',
                 horizontalalignment='left', verticalalignment='center', color="tab:blue")

    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance foe (m)", color=color2)
    ax2.plot(x_foe, y_foe, color=color2)
    # ax2.grid()
    ax2.tick_params(axis="y", labelcolor=color2)

    fig.tight_layout()
    plt.show()
    fig.savefig("plot/scenario_" + scena + ".png", dpi=1200)

elif scena == "12":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, _, _, _, _, _ \
        = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    istant_1 = np.argmax(vehicle_biblio)
    istant_2 = np.argmax(vehicle_terminal)

    fig, ax1 = plt.subplots()
    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance agent (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    ax1.plot(x_agent, y_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)

    ax1.annotate('start\ncars',
                 xy=(285, 1), xycoords='data',
                 xytext=(285, 750),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='center', verticalalignment='top')

    ax1.annotate('parked cars',
                 xy=(470, 1870), xycoords='data',
                 xytext=(510, 1870),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='left', verticalalignment='center')
    ax1.annotate(' ',
                 xy=(610, 1870), xycoords='data',
                 xytext=(570, 1870),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='right', verticalalignment='center')

    ax1.axvline(x=istant_1, linestyle="--")
    ax1.annotate('Biblioteca\nfills up', xy=(istant_1 - 5, 1850), xycoords='data',
                 horizontalalignment='right', verticalalignment='center', color="tab:blue")

    ax1.axvline(x=istant_2, linestyle="--")
    ax1.annotate('Terminal\nfills up', xy=(istant_2 + 5, 0), xycoords='data',
                 horizontalalignment='left', verticalalignment='center', color="tab:blue")

    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance foe (m)", color=color2)
    ax2.plot(x_foe, y_foe, color=color2)
    # ax2.grid()
    ax2.tick_params(axis="y", labelcolor=color2)

    fig.tight_layout()
    plt.show()
    fig.savefig("plot/scenario_" + scena + ".png", dpi=1200)

elif scena == "21":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, vehicle_multipiano, _, _, _, _ \
        = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    for i in range(step_agents[0][0], len(vehicle_multipiano)):
        if vehicle_multipiano[i] < 50:
            istant_1 = i
            break

    fig, ax1 = plt.subplots()
    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance agent (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    ax1.plot(x_agent, y_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)

    ax1.annotate('start\ncars',
                 xy=(500, 1), xycoords='data',
                 xytext=(500, 750),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='center', verticalalignment='top')

    ax1.annotate('parked cars',
                 xy=(650, 1870), xycoords='data',
                 xytext=(710, 1870),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='left', verticalalignment='center')
    ax1.annotate(' ',
                 xy=(860, 1870), xycoords='data',
                 xytext=(790, 1870),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='right', verticalalignment='center')

    ax1.axvline(x=istant_1, linestyle="--")
    ax1.annotate('Multipiano\nget empty', xy=(istant_1 + 5, 10), xycoords='data',
                 horizontalalignment='left', verticalalignment='center', color="tab:blue")

    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance foe (m)", color=color2)
    ax2.plot(x_foe, y_foe, color=color2)
    # ax2.grid()
    ax2.tick_params(axis="y", labelcolor=color2)

    fig.tight_layout()
    plt.show()
    fig.savefig("plot/scenario_" + scena + ".png", dpi=1200)

elif scena == "22":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, vehicle_multipiano, _, _, _, _ \
        = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    for i in range(step_agents[0][0], len(vehicle_multipiano)):
        if vehicle_multipiano[i] < 50:
            istant_1 = i
            break

    for j in range(step_agents[0][0], len(vehicle_terminal)):
        if vehicle_terminal[j] < 50:
            istant_2 = j
            break

    fig, ax1 = plt.subplots()
    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance agent (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    ax1.plot(x_agent, y_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)

    ax1.annotate('start\ncars',
                 xy=(520, 1), xycoords='data',
                 xytext=(520, 400),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='center', verticalalignment='top')

    ax1.annotate('parked cars',
                 xy=(600, 850), xycoords='data',
                 xytext=(620, 850),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='left', verticalalignment='center')
    ax1.annotate(' ',
                 xy=(670, 850), xycoords='data',
                 xytext=(650, 850),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='right', verticalalignment='center')

    ax1.axvline(x=istant_1, linestyle="--")
    ax1.annotate('Multipiano\nget empty', xy=(istant_1 - 1, 850), xycoords='data',
                 horizontalalignment='right', verticalalignment='center', color="tab:blue")

    ax1.axvline(x=istant_2, linestyle="--")
    ax1.annotate('Terminal\nget empty', xy=(istant_2 + 5, 10), xycoords='data',
                 horizontalalignment='left', verticalalignment='center', color="tab:blue")

    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance foe (m)", color=color2)
    ax2.plot(x_foe, y_foe, color=color2)
    # ax2.grid()
    ax2.tick_params(axis="y", labelcolor=color2)

    fig.tight_layout()
    plt.show()
    fig.savefig("plot/scenario_" + scena + ".png", dpi=1200)

elif scena == "23":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, vehicle_multipiano, _, _, _, _ \
        = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    for i in range(step_agents[0][0], len(vehicle_multipiano)):
        if vehicle_multipiano[i] < 50:
            istant_1 = i
            break

    for j in range(step_agents[0][0], len(vehicle_terminal)):
        if vehicle_terminal[j] < 50:
            istant_2 = j
            break

    for j in range(step_agents[0][0], len(vehicle_biblio)):
        if vehicle_biblio[j] < 50:
            istant_3 = j
            break

    fig, ax1 = plt.subplots()
    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance agent (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    ax1.plot(x_agent, y_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)

    ax1.annotate('start\ncars',
                 xy=(535, 15), xycoords='data',
                 xytext=(535, 200),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='center', verticalalignment='top')

    ax1.annotate('parked\ncars',
                 xy=(595, 590), xycoords='data',
                 xytext=(595, 400),
                 arrowprops=dict(facecolor='black', shrink=0.1),
                 horizontalalignment='center', verticalalignment='bottom')

    ax1.axvline(x=istant_1, linestyle="--")
    ax1.annotate('Multipiano\nget empty', xy=(istant_1 - 1, 600), xycoords='data',
                 horizontalalignment='right', verticalalignment='center', color="tab:blue")

    ax1.axvline(x=istant_2, linestyle="--")
    ax1.annotate('Terminal\nget empty', xy=(istant_2 + 1, 10), xycoords='data',
                 horizontalalignment='center', verticalalignment='center', color="tab:blue")

    ax1.axvline(x=istant_3, linestyle="--")
    ax1.annotate('Bibliotec\nget empty', xy=(istant_3 + 1, 600), xycoords='data',
                 horizontalalignment='left', verticalalignment='center', color="tab:blue")
    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance foe (m)", color=color2)
    ax2.plot(x_foe, y_foe, color=color2)
    # ax2.grid()
    ax2.tick_params(axis="y", labelcolor=color2)

    fig.tight_layout()
    plt.show()
    fig.savefig("plot/scenario_" + scena + ".png", dpi=1200)

elif scena == "31":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, \
    vehicle_multipiano, means_agent, variance_agent, means_foe, variance_foe = pickle.load(
        open("output_data/scenario_" + scena + ".p", "rb"))
    n_agents = len(distance_agents)
    istant_1 = np.argmax(vehicle_biblio)
    istant_2 = np.argmax(vehicle_terminal)
    istant_3 = np.argmax(vehicle_multipiano)

    # travel time plot
    x = [i for i in range(n_agents)]
    y_agent = [step[-1] - step[0] for step in step_agents]
    y_agent_mean = [np.mean(y_agent) for i in range(n_agents)]
    fig, (ax1, ax2) = plt.subplots(1, 2, constrained_layout=True, sharey='all', figsize=(8, 4))
    ax1.set_xlabel("Number Agent")
    ax1.set_ylabel("Travel Time (s)")
    ax1.set_title("Agents")
    ax1.stem(x, y_agent, linefmt="tab:red", markerfmt="ro")
    ax1.plot(x, y_agent_mean, color="tab:blue")
    ax1.xaxis.grid()
    ax1.yaxis.grid()

    n_foes = len(distance_foes)
    y_foe = [step[-1] - step[0] for step in step_foes]
    y_foe_mean = [np.mean(y_foe) for i in range(n_foes)]
    ax2.set_xlabel("Number Foe")
    ax2.set_title("Foes")

    ax2.stem(x, y_foe, linefmt="tab:green", markerfmt="go")
    ax2.plot(x, y_foe_mean, color="tab:blue")
    ax2.xaxis.grid()
    ax2.yaxis.grid()
    fig.suptitle("Travel Time - " + name_scenario, fontsize=16)
    plt.show()
    fig.savefig("plot/scenario_" + scena + "_travel_time.png", dpi=1200)

    # distance traveled plot
    y_agent_distance = [distance[-1] for distance in distance_agents]
    y_agent_mean_distance = [np.mean(y_agent_distance) for i in range(n_agents)]

    fig, (ax1, ax2) = plt.subplots(1, 2, constrained_layout=True, sharey='all', figsize=(8, 4))
    ax1.set_xlabel("Number Agent")
    ax1.set_ylabel("Distance Traveled (m)")
    ax1.set_title("Agents")
    ax1.stem(x, y_agent_distance, linefmt="tab:red", markerfmt="ro")
    ax1.plot(x, y_agent_mean_distance, color="tab:blue")
    ax1.xaxis.grid()
    ax1.yaxis.grid()

    y_foe_distance = [distance[-1] for distance in distance_foes]
    y_foe_mean_distance = [np.mean(y_foe_distance) for i in range(n_foes)]
    ax2.set_xlabel("Number Foe")
    ax2.set_title("Foes")
    ax2.stem(x, y_foe_distance, linefmt="tab:green", markerfmt="go")
    ax2.plot(x, y_foe_mean_distance, color="tab:blue")
    ax2.xaxis.grid()
    ax2.yaxis.grid()
    fig.suptitle("Distance Traveled - " + name_scenario, fontsize=16)
    plt.show()
    fig.savefig("plot/scenario_" + scena + "_distance_traveled.png", dpi=1200)

    # bar chart
    labels = ['<100', '100-200', '200-300', '300-400', '>400']
    y_agent = np.array(y_agent)
    agent_1 = y_agent[y_agent < 100]
    agent_2 = y_agent[(y_agent >= 100) & (y_agent < 200)]
    agent_3 = y_agent[(y_agent >= 200) & (y_agent < 300)]
    agent_4 = y_agent[(y_agent >= 300) & (y_agent < 400)]
    agent_5 = y_agent[y_agent >= 400]
    set_agents = [len(agent_1), len(agent_2), len(agent_3), len(agent_4), len(agent_5)]

    y_foe = np.array(y_foe)
    foe_1 = y_foe[y_foe < 100]
    foe_2 = y_foe[(y_foe >= 100) & (y_foe < 200)]
    foe_3 = y_foe[(y_foe >= 200) & (y_foe < 300)]
    foe_4 = y_foe[(y_foe >= 300) & (y_foe < 400)]
    foe_5 = y_foe[y_foe >= 400]
    set_foes = [len(foe_1), len(foe_2), len(foe_3), len(foe_4), len(foe_5)]

    x = np.arange(len(labels))  # the label locations
    width = 0.35  # the width of the bars

    fig, ax = plt.subplots()
    rects1 = ax.bar(x - width / 2, set_agents, width, label='Agents', color="tab:red")
    rects2 = ax.bar(x + width / 2, set_foes, width, label='Foe', color="tab:green")

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel('Number of vehicles')
    ax.set_title('Number of vehicles by type and travel time')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    ax.xaxis.grid()
    ax.yaxis.grid()

    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)

    fig.tight_layout()

    plt.show()
    fig.savefig("plot/scenario_" + scena + "_bar_chart.png", dpi=1200)

    # mean and variance
    fig, ax1 = plt.subplots()

    x = np.arange(0, len(means_agent), 1)
    color1 = "tab:red"
    ax1.set_xlabel("Step (s)")
    ax1.set_ylabel("Mean distance(m)")
    ax1.set(title="Mean distance traveled - " + name_scenario)

    ax1.plot(x, means_agent, color=color1, label='Agent')
    color2 = "tab:green"
    ax1.plot(x, means_foe, color=color2, label='Foe')
    ax1.xaxis.grid()
    ax1.yaxis.grid()
    ax1.legend(loc='upper left')

    plt.show()
    fig.savefig("plot/scenario_" + scena + "_mean_distance.png", dpi=1200)

    fig, ax1 = plt.subplots()
    ax1.set_xlabel("Step (s)")
    ax1.set_ylabel("Variance distance (m)")
    ax1.set(title="Variance distance traveled - " + name_scenario)

    ax1.plot(x, variance_agent, color=color1, label='Agent')
    color2 = "tab:green"
    ax1.plot(x, variance_foe, color=color2, label='Foe')
    ax1.xaxis.grid()
    ax1.yaxis.grid()
    ax1.legend(loc='upper left')

    plt.show()
    fig.savefig("plot/scenario_" + scena + "_variance_distance.png", dpi=1200)

    for i in range(n_agents):
        print("step", step_agents[i])
        print("distance", distance_agents[i])
    print("media", means_agent)
