import math
import pickle

import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np

scena = "11"

if scena == "00":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, vehicle_multipiano = pickle.load(
        open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]

    fig = plt.figure(figsize=(11, 6.2), dpi=200, tight_layout=True)
    gs = gridspec.GridSpec(3, 2)

    ax1 = fig.add_subplot(gs[:, 0])

    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance Crowd Car (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    x_data_agent, y_data_agent = [], []
    line1, = ax1.plot(x_data_agent, y_data_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)
    ax1.set_ylim(-5 / 100 * max(y_agent), max(y_agent) + 5 / 100 * max(y_agent))
    x_min = min(x_agent[0], x_foe[0])
    x_max = max(x_agent[-1], x_foe[-1])
    x_inf = math.floor(x_min - 5 / 100 * (x_max - x_min))
    x_sup = math.ceil(x_max + 5 / 100 * (x_max - x_min))
    ax1.set_xlim(x_inf, x_sup)

    x_data_foe, y_data_foe = [], []
    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance Sumo Car (m)", color=color2)
    line2, = ax2.plot(x_data_foe, y_data_foe, color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)
    ax2.set_ylim(-5 / 100 * max(y_foe), max(y_foe) + 5 / 100 * max(y_foe))

    ax3 = fig.add_subplot(gs[0, 1])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 1])

    ax5.set_xlabel("Time (s)")
    ax3.set(title="Occupancy percentage")

    ax3.set_ylabel("Biblioteca(%)")
    x_data_biblio, y_data_biblio = [], []
    line3, = ax3.plot(x_data_biblio, y_data_biblio)
    ax3.xaxis.grid()
    ax3.yaxis.grid()
    ax3.tick_params(axis="y")
    ax3.set_ylim(-15, 115)
    ax3.set_xlim(x_inf, x_sup)

    ax4.set_ylabel("Terminal(%)")
    x_data_terminal, y_data_terminal = [], []
    line4, = ax4.plot(x_data_terminal, y_data_terminal)
    ax4.xaxis.grid()
    ax4.yaxis.grid()
    ax4.tick_params(axis="y")
    ax4.set_ylim(-15, 115)
    ax4.set_xlim(x_inf, x_sup)

    ax5.set_ylabel("Multipiano(%)")
    x_data_multipiano, y_data_multipiano = [], []
    line5, = ax5.plot(x_data_multipiano, y_data_multipiano)
    ax5.xaxis.grid()
    ax5.yaxis.grid()
    ax5.tick_params(axis="y")
    ax5.set_ylim(-15, 115)
    ax5.set_xlim(x_inf, x_sup)


    def data_gen():
        i = 0
        j = 0
        for cnt in range(x_inf, x_sup):
            if i < len(x_agent) and cnt == x_agent[i]:
                ret_i = i
                i += 1
            else:
                ret_i = None
            if j < len(x_foe) and cnt == x_foe[j]:
                ret_j = j
                j += 1
            else:
                ret_j = None
            yield ret_i, ret_j, cnt


    # plt.show()
    # animation
    def run(i):
        print(i)
        if i[0] is not None:
            x_data_agent.append(x_agent[i[0]])
            y_data_agent.append(y_agent[i[0]])

        line1.set_data(x_data_agent, y_data_agent)

        if i[1] is not None:
            x_data_foe.append(x_foe[i[1]])
            y_data_foe.append(y_foe[i[1]])

        line2.set_data(x_data_foe, y_data_foe)
        x_data_biblio.append(i[2])
        y_data_biblio.append(vehicle_biblio[i[2]] / 50 * 100)
        line3.set_data(x_data_biblio, y_data_biblio)

        x_data_terminal.append(i[2])
        y_data_terminal.append(vehicle_terminal[i[2]] / 50 * 100)
        line4.set_data(x_data_terminal, y_data_terminal)

        x_data_multipiano.append(i[2])
        y_data_multipiano.append(vehicle_multipiano[i[2]] / 50 * 100)
        line5.set_data(x_data_multipiano, y_data_multipiano)

        if i[2] == 5:
            ax1.annotate('starting\ncars',
                         xy=(5, 10), xycoords='data',
                         xytext=(5, 200),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='top')

        if i[2] == 62:
            ax1.annotate('parked\ncars',
                         xy=(62, 600), xycoords='data',
                         xytext=(62, 400),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='bottom')

        return [line1, line2]


    ani = animation.FuncAnimation(fig, run, data_gen, interval=200, blit=True, save_count=x_sup - x_inf + 1)
    ani.save("plot/gif/scenario_" + scena + ".gif", writer='pillow')

    # plt.show()
    fig.savefig("plot/scenario_" + scena + ".png")
    fig.savefig("plot/scenario_" + scena + ".pdf")


elif scena == "01":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, \
    vehicle_multipiano, _, _, _, _ = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]

    fig = plt.figure(figsize=(11, 6.2), dpi=200, tight_layout=True)
    gs = gridspec.GridSpec(3, 2)

    ax1 = fig.add_subplot(gs[:, 0])

    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance Crowd Car (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    x_data_agent, y_data_agent = [], []
    line1, = ax1.plot(x_data_agent, y_data_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)
    ax1.set_ylim(-5 / 100 * max(y_agent), max(y_agent) + 5 / 100 * max(y_agent))
    x_min = min(x_agent[0], x_foe[0])
    x_max = max(x_agent[-1], x_foe[-1])
    x_inf = math.floor(x_min - 5 / 100 * (x_max - x_min))
    x_sup = math.ceil(x_max + 5 / 100 * (x_max - x_min))
    ax1.set_xlim(x_inf, x_sup)

    x_data_foe, y_data_foe = [], []
    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance Sumo Car (m)", color=color2)
    line2, = ax2.plot(x_data_foe, y_data_foe, color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)
    ax2.set_ylim(-5 / 100 * max(y_foe), max(y_foe) + 5 / 100 * max(y_foe))

    ax3 = fig.add_subplot(gs[0, 1])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 1])

    ax5.set_xlabel("Time (s)")
    ax3.set(title="Occupancy percentage")

    ax3.set_ylabel("Biblioteca(%)")
    x_data_biblio, y_data_biblio = [], []
    line3, = ax3.plot(x_data_biblio, y_data_biblio)
    ax3.xaxis.grid()
    ax3.yaxis.grid()
    ax3.tick_params(axis="y")
    ax3.set_ylim(-15, 115)
    ax3.set_xlim(x_inf, x_sup)

    ax4.set_ylabel("Terminal(%)")
    x_data_terminal, y_data_terminal = [], []
    line4, = ax4.plot(x_data_terminal, y_data_terminal)
    ax4.xaxis.grid()
    ax4.yaxis.grid()
    ax4.tick_params(axis="y")
    ax4.set_ylim(-15, 115)
    ax4.set_xlim(x_inf, x_sup)

    ax5.set_ylabel("Multipiano(%)")
    x_data_multipiano, y_data_multipiano = [], []
    line5, = ax5.plot(x_data_multipiano, y_data_multipiano)
    ax5.xaxis.grid()
    ax5.yaxis.grid()
    ax5.tick_params(axis="y")
    ax5.set_ylim(-15, 115)
    ax5.set_xlim(x_inf, x_sup)


    def data_gen():
        i = 0
        j = 0
        for cnt in range(x_inf, x_sup):
            if i < len(x_agent) and cnt == x_agent[i]:
                ret_i = i
                i += 1
            else:
                ret_i = None
            if j < len(x_foe) and cnt == x_foe[j]:
                ret_j = j
                j += 1
            else:
                ret_j = None
            yield ret_i, ret_j, cnt


    def run(i):
        print(i)
        if i[0] is not None:
            x_data_agent.append(x_agent[i[0]])
            y_data_agent.append(y_agent[i[0]])

        line1.set_data(x_data_agent, y_data_agent)

        if i[1] is not None:
            x_data_foe.append(x_foe[i[1]])
            y_data_foe.append(y_foe[i[1]])

        line2.set_data(x_data_foe, y_data_foe)

        x_data_biblio.append(i[2])
        y_data_biblio.append(vehicle_biblio[i[2]] / 50 * 100)
        line3.set_data(x_data_biblio, y_data_biblio)

        x_data_terminal.append(i[2])
        y_data_terminal.append(vehicle_terminal[i[2]] / 50 * 100)
        line4.set_data(x_data_terminal, y_data_terminal)

        x_data_multipiano.append(i[2])
        y_data_multipiano.append(vehicle_multipiano[i[2]] / 50 * 100)
        line5.set_data(x_data_multipiano, y_data_multipiano)

        if i[2] == x_agent[0]:
            ax1.annotate('starting\ncars',
                         xy=(x_agent[0], y_agent[0] + 30), xycoords='data',
                         xytext=(x_agent[0], y_agent[0] + 200),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='top')

        if i[2] == x_foe[-1] - 5:
            ax1.annotate('parked\ncars',
                         xy=(x_agent[-1] + 5, y_agent[-1]), xycoords='data',
                         xytext=(x_agent[-1] + (x_foe[-1] - x_agent[-1]) / 2, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')
            ax1.annotate(' ',
                         xy=(x_foe[-1] - 5, y_agent[-1]), xycoords='data',
                         xytext=(x_foe[-1] - 30, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')

        return [line1, line2, line3, line4, line5]


    ani = animation.FuncAnimation(fig, run, data_gen, interval=200, blit=True, save_count=x_sup - x_inf + 1)
    ani.save("plot/gif/scenario_" + scena + ".gif", writer='pillow')

    # plt.show()
    fig.savefig("plot/scenario_" + scena + ".png")
    fig.savefig("plot/scenario_" + scena + ".pdf")


elif scena == "02":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, \
    vehicle_multipiano, _, _, _, _ = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]

    fig = plt.figure(figsize=(11, 6.2), dpi=200, tight_layout=True)
    gs = gridspec.GridSpec(3, 2)
    ax1 = fig.add_subplot(gs[:, 0])

    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance Crowd Car (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    x_data_agent, y_data_agent = [], []
    line1, = ax1.plot(x_data_agent, y_data_agent, color=color1)
    ax1.xaxis.grid(10)
    ax1.tick_params(axis="y", labelcolor=color1)
    ax1.set_ylim(-5 / 100 * max(y_agent), max(y_agent) + 5 / 100 * max(y_agent))
    x_min = min(x_agent[0], x_foe[0])
    x_max = max(x_agent[-1], x_foe[-1])
    x_inf = math.floor(x_min - 5 / 100 * (x_max - x_min))
    x_sup = math.ceil(x_max + 5 / 100 * (x_max - x_min))
    ax1.set_xlim(x_inf, x_sup)

    x_data_foe, y_data_foe = [], []
    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance Sumo Car (m)", color=color2)
    line2, = ax2.plot(x_data_foe, y_data_foe, color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)
    ax2.set_ylim(-5 / 100 * max(y_foe), max(y_foe) + 5 / 100 * max(y_foe))

    ax3 = fig.add_subplot(gs[0, 1])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 1])

    ax5.set_xlabel("Time (s)")
    ax3.set(title="Occupancy percentage")

    ax3.set_ylabel("Biblioteca(%)")
    x_data_biblio, y_data_biblio = [], []
    line3, = ax3.plot(x_data_biblio, y_data_biblio)
    ax3.xaxis.grid()
    ax3.yaxis.grid()
    ax3.tick_params(axis="y")
    ax3.set_ylim(-15, 115)
    ax3.set_xlim(x_inf, x_sup)

    ax4.set_ylabel("Terminal(%)")
    x_data_terminal, y_data_terminal = [], []
    line4, = ax4.plot(x_data_terminal, y_data_terminal)
    ax4.xaxis.grid()
    ax4.yaxis.grid()
    ax4.tick_params(axis="y")
    ax4.set_ylim(-15, 115)
    ax4.set_xlim(x_inf, x_sup)

    ax5.set_ylabel("Multipiano(%)")
    x_data_multipiano, y_data_multipiano = [], []
    line5, = ax5.plot(x_data_multipiano, y_data_multipiano)
    ax5.xaxis.grid()
    ax5.yaxis.grid()
    ax5.tick_params(axis="y")
    ax5.set_ylim(-15, 115)
    ax5.set_xlim(x_inf, x_sup)


    def data_gen():
        i = 0
        j = 0
        for cnt in range(x_inf, x_sup):
            if i < len(x_agent) and cnt == x_agent[i]:
                ret_i = i
                i += 1
            else:
                ret_i = None
            if j < len(x_foe) and cnt == x_foe[j]:
                ret_j = j
                j += 1
            else:
                ret_j = None
            yield ret_i, ret_j, cnt


    def run(i):
        print(i)
        if i[0] is not None:
            x_data_agent.append(x_agent[i[0]])
            y_data_agent.append(y_agent[i[0]])

        line1.set_data(x_data_agent, y_data_agent)

        if i[1] is not None:
            x_data_foe.append(x_foe[i[1]])
            y_data_foe.append(y_foe[i[1]])

        line2.set_data(x_data_foe, y_data_foe)

        x_data_biblio.append(i[2])
        y_data_biblio.append(vehicle_biblio[i[2]] / 50 * 100)
        line3.set_data(x_data_biblio, y_data_biblio)

        x_data_terminal.append(i[2])
        y_data_terminal.append(vehicle_terminal[i[2]] / 50 * 100)
        line4.set_data(x_data_terminal, y_data_terminal)

        x_data_multipiano.append(i[2])
        y_data_multipiano.append(vehicle_multipiano[i[2]] / 50 * 100)
        line5.set_data(x_data_multipiano, y_data_multipiano)

        if i[2] == x_agent[0]:
            ax1.annotate('starting\ncars',
                         xy=(x_agent[0], y_agent[0] + 30), xycoords='data',
                         xytext=(x_agent[0], y_agent[0] + 350),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='top')

        if i[2] == x_foe[-1] - 5:
            ax1.annotate('parked\ncars',
                         xy=(x_agent[-1] + 5, y_agent[-1]), xycoords='data',
                         xytext=(x_agent[-1] + (x_foe[-1] - x_agent[-1]) / 2, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')
            ax1.annotate(' ',
                         xy=(x_foe[-1] - 5, y_agent[-1]), xycoords='data',
                         xytext=(x_foe[-1] - 60, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')

        return [line1, line2, line3, line4, line5]


    ani = animation.FuncAnimation(fig, run, data_gen, interval=200, blit=True, save_count=x_sup - x_inf + 1)
    ani.save("plot/gif/scenario_" + scena + ".gif", writer='pillow')

    # plt.show()
    fig.savefig("plot/scenario_" + scena + ".png")
    fig.savefig("plot/scenario_" + scena + ".pdf")


elif scena == "11":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, \
    vehicle_multipiano, _, _, _, _ = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))

    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    istant = np.argmax(vehicle_biblio)

    fig = plt.figure(figsize=(11, 6.2), dpi=200, tight_layout=True)
    gs = gridspec.GridSpec(3, 2)

    ax1 = fig.add_subplot(gs[:, 0])

    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance Crowd Car (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    x_data_agent, y_data_agent = [], []
    line1, = ax1.plot(x_data_agent, y_data_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)
    ax1.set_ylim(-5 / 100 * max(y_agent), max(y_agent) + 5 / 100 * max(y_agent))
    x_min = min(x_agent[0], x_foe[0])
    x_max = max(x_agent[-1], x_foe[-1])
    x_inf = math.floor(x_min - 5 / 100 * (x_max - x_min))
    x_sup = math.ceil(x_max + 5 / 100 * (x_max - x_min))
    ax1.set_xlim(x_inf, x_sup)

    x_data_foe, y_data_foe = [], []
    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance Sumo Car (m)", color=color2)
    line2, = ax2.plot(x_data_foe, y_data_foe, color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)
    ax2.set_ylim(-5 / 100 * max(y_foe), max(y_foe) + 5 / 100 * max(y_foe))

    ax3 = fig.add_subplot(gs[0, 1])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 1])

    ax5.set_xlabel("Time (s)")
    ax3.set(title="Occupancy percentage")

    ax3.set_ylabel("Biblioteca(%)")
    x_data_biblio, y_data_biblio = [], []
    line3, = ax3.plot(x_data_biblio, y_data_biblio)
    ax3.xaxis.grid()
    ax3.yaxis.grid()
    ax3.tick_params(axis="y")
    ax3.set_ylim(-15, 115)
    ax3.set_xlim(x_inf, x_sup)

    ax4.set_ylabel("Terminal(%)")
    x_data_terminal, y_data_terminal = [], []
    line4, = ax4.plot(x_data_terminal, y_data_terminal)
    ax4.xaxis.grid()
    ax4.yaxis.grid()
    ax4.tick_params(axis="y")
    ax4.set_ylim(-15, 115)
    ax4.set_xlim(x_inf, x_sup)

    ax5.set_ylabel("Multipiano(%)")
    x_data_multipiano, y_data_multipiano = [], []
    line5, = ax5.plot(x_data_multipiano, y_data_multipiano)
    ax5.xaxis.grid()
    ax5.yaxis.grid()
    ax5.tick_params(axis="y")
    ax5.set_ylim(-15, 115)
    ax5.set_xlim(x_inf, x_sup)


    def data_gen():
        i = 0
        j = 0
        for cnt in range(x_inf, x_sup):
            if i < len(x_agent) and cnt == x_agent[i]:
                ret_i = i
                i += 1
            else:
                ret_i = None
            if j < len(x_foe) and cnt == x_foe[j]:
                ret_j = j
                j += 1
            else:
                ret_j = None
            yield ret_i, ret_j, cnt


    def run(i):
        print(i)
        if i[0] is not None:
            x_data_agent.append(x_agent[i[0]])
            y_data_agent.append(y_agent[i[0]])

        line1.set_data(x_data_agent, y_data_agent)

        if i[1] is not None:
            x_data_foe.append(x_foe[i[1]])
            y_data_foe.append(y_foe[i[1]])

        line2.set_data(x_data_foe, y_data_foe)

        x_data_biblio.append(i[2])
        y_data_biblio.append(vehicle_biblio[i[2]] / 50 * 100)
        line3.set_data(x_data_biblio, y_data_biblio)

        x_data_terminal.append(i[2])
        y_data_terminal.append(vehicle_terminal[i[2]] / 50 * 100)
        line4.set_data(x_data_terminal, y_data_terminal)

        x_data_multipiano.append(i[2])
        y_data_multipiano.append(vehicle_multipiano[i[2]] / 50 * 100)
        line5.set_data(x_data_multipiano, y_data_multipiano)

        if i[2] == x_agent[0]:
            ax1.annotate('starting\ncars',
                         xy=(x_agent[0], y_agent[0] + 30), xycoords='data',
                         xytext=(x_agent[0], y_agent[0] + 200),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='top')

        if i[2] == x_foe[-1] - 5:
            ax1.annotate('parked\ncars',
                         xy=(x_agent[-1] + 5, y_agent[-1]), xycoords='data',
                         xytext=(x_agent[-1] + (x_foe[-1] - x_agent[-1]) / 2, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')
            ax1.annotate(' ',
                         xy=(x_foe[-1] - 5, y_agent[-1]), xycoords='data',
                         xytext=(
                             x_foe[-1] - (x_agent[-1] + (x_foe[-1] - x_agent[-1]) / 2) + (x_agent[-1] + 5),
                             y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')

        if i[2] == istant:
            ax1.axvline(x=istant, linestyle="--")
            ax1.annotate('Biblioteca\nfills up', xy=(istant + 1, 0), xycoords='data',
                         horizontalalignment='left', verticalalignment='center', color="tab:blue")

        return [line1, line2, line3, line4, line5]


    ani = animation.FuncAnimation(fig, run, data_gen, interval=200, blit=True, save_count=x_sup - x_inf + 1)
    ani.save("plot/gif/scenario_" + scena + ".gif", writer='pillow')

    # plt.show()
    fig.savefig("plot/scenario_" + scena + ".png")
    fig.savefig("plot/scenario_" + scena + ".pdf")


elif scena == "12":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, \
    vehicle_multipiano, _, _, _, _ = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    istant_1 = np.argmax(vehicle_biblio)
    istant_2 = np.argmax(vehicle_terminal)

    fig = plt.figure(figsize=(12, 6.72), dpi=200, tight_layout=True)
    gs = gridspec.GridSpec(3, 2)
    ax1 = fig.add_subplot(gs[:, 0])

    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance Crowd Car (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    x_data_agent, y_data_agent = [], []
    line1, = ax1.plot(x_data_agent, y_data_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)
    ax1.set_ylim(-5 / 100 * max(y_agent), max(y_agent) + 5 / 100 * max(y_agent))
    x_min = min(x_agent[0], x_foe[0])
    x_max = max(x_agent[-1], x_foe[-1])
    x_inf = math.floor(x_min - 5 / 100 * (x_max - x_min))
    x_sup = math.ceil(x_max + 5 / 100 * (x_max - x_min))
    ax1.set_xlim(x_inf, x_sup)

    x_data_foe, y_data_foe = [], []
    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance Sumo Car (m)", color=color2)
    line2, = ax2.plot(x_data_foe, y_data_foe, color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)
    ax2.set_ylim(-5 / 100 * max(y_foe), max(y_foe) + 5 / 100 * max(y_foe))

    ax3 = fig.add_subplot(gs[0, 1])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 1])

    ax5.set_xlabel("Time (s)")
    ax3.set(title="Occupancy percentage")

    ax3.set_ylabel("Biblioteca(%)")
    x_data_biblio, y_data_biblio = [], []
    line3, = ax3.plot(x_data_biblio, y_data_biblio)
    ax3.xaxis.grid()
    ax3.yaxis.grid()
    ax3.tick_params(axis="y")
    ax3.set_ylim(-15, 115)
    ax3.set_xlim(x_inf, x_sup)

    ax4.set_ylabel("Terminal(%)")
    x_data_terminal, y_data_terminal = [], []
    line4, = ax4.plot(x_data_terminal, y_data_terminal)
    ax4.xaxis.grid()
    ax4.yaxis.grid()
    ax4.tick_params(axis="y")
    ax4.set_ylim(-15, 115)
    ax4.set_xlim(x_inf, x_sup)

    ax5.set_ylabel("Multipiano(%)")
    x_data_multipiano, y_data_multipiano = [], []
    line5, = ax5.plot(x_data_multipiano, y_data_multipiano)
    ax5.xaxis.grid()
    ax5.yaxis.grid()
    ax5.tick_params(axis="y")
    ax5.set_ylim(-15, 115)
    ax5.set_xlim(x_inf, x_sup)


    def data_gen():
        i = 0
        j = 0
        for cnt in range(x_inf, x_sup):
            if i < len(x_agent) and cnt == x_agent[i]:
                ret_i = i
                i += 1
            else:
                ret_i = None
            if j < len(x_foe) and cnt == x_foe[j]:
                ret_j = j
                j += 1
            else:
                ret_j = None
            yield ret_i, ret_j, cnt


    def run(i):
        print(i)
        if i[0] is not None:
            x_data_agent.append(x_agent[i[0]])
            y_data_agent.append(y_agent[i[0]])

        line1.set_data(x_data_agent, y_data_agent)

        if i[1] is not None:
            x_data_foe.append(x_foe[i[1]])
            y_data_foe.append(y_foe[i[1]])

        line2.set_data(x_data_foe, y_data_foe)

        x_data_biblio.append(i[2])
        y_data_biblio.append(vehicle_biblio[i[2]] / 50 * 100)
        line3.set_data(x_data_biblio, y_data_biblio)

        x_data_terminal.append(i[2])
        y_data_terminal.append(vehicle_terminal[i[2]] / 50 * 100)
        line4.set_data(x_data_terminal, y_data_terminal)

        x_data_multipiano.append(i[2])
        y_data_multipiano.append(vehicle_multipiano[i[2]] / 50 * 100)
        line5.set_data(x_data_multipiano, y_data_multipiano)

        if i[2] == x_agent[0]:
            ax1.annotate('starting\ncars',
                         xy=(x_agent[0], y_agent[0] + 30), xycoords='data',
                         xytext=(x_agent[0], y_agent[0] + 500),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='top')

        if i[2] == x_foe[-1] - 5:
            ax1.annotate('parked\ncars',
                         xy=(x_agent[-1] + 5, y_agent[-1]), xycoords='data',
                         xytext=(x_agent[-1] + (x_foe[-1] - x_agent[-1]) / 2, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')
            ax1.annotate(' ',
                         xy=(x_foe[-1] - 5, y_agent[-1]), xycoords='data',
                         xytext=(x_foe[-1] - ((x_foe[-1] - x_agent[-1]) / 2) + 5, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')

        if i[2] == istant_1:
            ax1.axvline(x=istant_1, linestyle="--")
            ax1.annotate('Biblioteca\nfills up', xy=(istant_1 - 5, 1850), xycoords='data',
                         horizontalalignment='right', verticalalignment='center', color="tab:blue")

        if i[2] == istant_2:
            ax1.axvline(x=istant_2, linestyle="--")
            ax1.annotate('Terminal\nfills up', xy=(istant_2 + 5, 0), xycoords='data',
                         horizontalalignment='left', verticalalignment='center', color="tab:blue")

        return [line1, line2, line3, line4, line5]


    ani = animation.FuncAnimation(fig, run, data_gen, interval=200, blit=True, save_count=x_sup - x_inf + 1)
    ani.save("plot/gif/scenario_" + scena + ".gif", writer='pillow')

    # plt.show()
    fig.savefig("plot/scenario_" + scena + ".png")
    fig.savefig("plot/scenario_" + scena + ".pdf")


elif scena == "21":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, \
    vehicle_multipiano, _, _, _, _ = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
    x_agent = step_agents[0]
    y_agent = distance_agents[0]
    x_foe = step_foes[0]
    y_foe = distance_foes[0]
    for i in range(step_agents[0][0], len(vehicle_multipiano)):
        if vehicle_multipiano[i] < 50:
            istant_1 = i
            break

    fig = plt.figure(figsize=(12, 6.72), dpi=200, tight_layout=True)
    gs = gridspec.GridSpec(3, 2)
    ax1 = fig.add_subplot(gs[:, 0])

    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance Crowd Car (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    x_data_agent, y_data_agent = [], []
    line1, = ax1.plot(x_data_agent, y_data_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)
    ax1.set_ylim(-5 / 100 * max(y_agent), max(y_agent) + 5 / 100 * max(y_agent))
    x_min = min(x_agent[0], x_foe[0])
    x_max = max(x_agent[-1], x_foe[-1])
    x_inf = math.floor(x_min - 5 / 100 * (x_max - x_min))
    x_sup = math.ceil(x_max + 5 / 100 * (x_max - x_min))
    ax1.set_xlim(x_inf, x_sup)

    x_data_foe, y_data_foe = [], []
    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance Sumo Car (m)", color=color2)
    line2, = ax2.plot(x_data_foe, y_data_foe, color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)
    ax2.set_ylim(-5 / 100 * max(y_foe), max(y_foe) + 5 / 100 * max(y_foe))

    ax3 = fig.add_subplot(gs[0, 1])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 1])

    ax5.set_xlabel("Time (s)")
    ax3.set(title="Occupancy percentage")

    ax3.set_ylabel("Biblioteca(%)")
    x_data_biblio, y_data_biblio = [], []
    line3, = ax3.plot(x_data_biblio, y_data_biblio)
    ax3.xaxis.grid()
    ax3.yaxis.grid()
    ax3.tick_params(axis="y")
    ax3.set_ylim(-15, 115)
    ax3.set_xlim(x_inf, x_sup)

    ax4.set_ylabel("Terminal(%)")
    x_data_terminal, y_data_terminal = [], []
    line4, = ax4.plot(x_data_terminal, y_data_terminal)
    ax4.xaxis.grid()
    ax4.yaxis.grid()
    ax4.tick_params(axis="y")
    ax4.set_ylim(-15, 115)
    ax4.set_xlim(x_inf, x_sup)

    ax5.set_ylabel("Multipiano(%)")
    x_data_multipiano, y_data_multipiano = [], []
    line5, = ax5.plot(x_data_multipiano, y_data_multipiano)
    ax5.xaxis.grid()
    ax5.yaxis.grid()
    ax5.tick_params(axis="y")
    ax5.set_ylim(-15, 115)
    ax5.set_xlim(x_inf, x_sup)


    def data_gen():
        i = 0
        j = 0
        for cnt in range(x_inf, x_sup):
            if i < len(x_agent) and cnt == x_agent[i]:
                ret_i = i
                i += 1
            else:
                ret_i = None
            if j < len(x_foe) and cnt == x_foe[j]:
                ret_j = j
                j += 1
            else:
                ret_j = None
            yield ret_i, ret_j, cnt


    def run(i):
        print(i)
        if i[0] is not None:
            x_data_agent.append(x_agent[i[0]])
            y_data_agent.append(y_agent[i[0]])

        line1.set_data(x_data_agent, y_data_agent)

        if i[1] is not None:
            x_data_foe.append(x_foe[i[1]])
            y_data_foe.append(y_foe[i[1]])

        line2.set_data(x_data_foe, y_data_foe)

        x_data_biblio.append(i[2])
        y_data_biblio.append(vehicle_biblio[i[2]] / 50 * 100)
        line3.set_data(x_data_biblio, y_data_biblio)

        x_data_terminal.append(i[2])
        y_data_terminal.append(vehicle_terminal[i[2]] / 50 * 100)
        line4.set_data(x_data_terminal, y_data_terminal)

        x_data_multipiano.append(i[2])
        y_data_multipiano.append(vehicle_multipiano[i[2]] / 50 * 100)
        line5.set_data(x_data_multipiano, y_data_multipiano)

        if i[2] == x_agent[0]:
            ax1.annotate('starting\ncars',
                         xy=(x_agent[0], y_agent[0] + 30), xycoords='data',
                         xytext=(x_agent[0], y_agent[0] + 500),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='top')

        if i[2] == x_foe[-1] - 5:
            ax1.annotate('parked\ncars',
                         xy=(x_agent[-1] + 5, y_agent[-1]), xycoords='data',
                         xytext=(x_agent[-1] + (x_foe[-1] - x_agent[-1]) / 2, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')
            ax1.annotate(' ',
                         xy=(x_foe[-1] - 5, y_agent[-1]), xycoords='data',
                         xytext=(x_foe[-1] - ((x_foe[-1] - x_agent[-1]) / 2) + 5, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')

        if i[2] == istant_1:
            ax1.axvline(x=istant_1, linestyle="--")
            ax1.annotate('Multipiano\nget empty', xy=(istant_1 + 5, 10), xycoords='data',
                         horizontalalignment='left', verticalalignment='center', color="tab:blue")

        return [line1, line2, line3, line4, line5]


    ani = animation.FuncAnimation(fig, run, data_gen, interval=200, blit=True, save_count=x_sup - x_inf + 1)
    ani.save("plot/gif/scenario_" + scena + ".gif", writer='pillow')

    # plt.show()
    fig.savefig("plot/scenario_" + scena + ".png")
    fig.savefig("plot/scenario_" + scena + ".pdf")


elif scena == "22":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, \
    vehicle_multipiano, _, _, _, _ = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
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

    fig = plt.figure(figsize=(12, 6.72), dpi=200, tight_layout=True)
    gs = gridspec.GridSpec(3, 2)
    ax1 = fig.add_subplot(gs[:, 0])

    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance Crowd Car (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    x_data_agent, y_data_agent = [], []
    line1, = ax1.plot(x_data_agent, y_data_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)
    ax1.set_ylim(-5 / 100 * max(y_agent), max(y_agent) + 5 / 100 * max(y_agent))
    x_min = min(x_agent[0], x_foe[0])
    x_max = max(x_agent[-1], x_foe[-1])
    x_inf = math.floor(x_min - 5 / 100 * (x_max - x_min))
    x_sup = math.ceil(x_max + 5 / 100 * (x_max - x_min))
    ax1.set_xlim(x_inf, x_sup)

    x_data_foe, y_data_foe = [], []
    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance Sumo Car (m)", color=color2)
    line2, = ax2.plot(x_data_foe, y_data_foe, color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)
    ax2.set_ylim(-5 / 100 * max(y_foe), max(y_foe) + 5 / 100 * max(y_foe))

    ax3 = fig.add_subplot(gs[0, 1])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 1])

    ax5.set_xlabel("Time (s)")
    ax3.set(title="Occupancy percentage")

    ax3.set_ylabel("Biblioteca(%)")
    x_data_biblio, y_data_biblio = [], []
    line3, = ax3.plot(x_data_biblio, y_data_biblio)
    ax3.xaxis.grid()
    ax3.yaxis.grid()
    ax3.tick_params(axis="y")
    ax3.set_ylim(-15, 115)
    ax3.set_xlim(x_inf, x_sup)

    ax4.set_ylabel("Terminal(%)")
    x_data_terminal, y_data_terminal = [], []
    line4, = ax4.plot(x_data_terminal, y_data_terminal)
    ax4.xaxis.grid()
    ax4.yaxis.grid()
    ax4.tick_params(axis="y")
    ax4.set_ylim(-15, 115)
    ax4.set_xlim(x_inf, x_sup)

    ax5.set_ylabel("Multipiano(%)")
    x_data_multipiano, y_data_multipiano = [], []
    line5, = ax5.plot(x_data_multipiano, y_data_multipiano)
    ax5.xaxis.grid()
    ax5.yaxis.grid()
    ax5.tick_params(axis="y")
    ax5.set_ylim(-15, 115)
    ax5.set_xlim(x_inf, x_sup)


    def data_gen():
        i = 0
        j = 0
        for cnt in range(x_inf, x_sup):
            if i < len(x_agent) and cnt == x_agent[i]:
                ret_i = i
                i += 1
            else:
                ret_i = None
            if j < len(x_foe) and cnt == x_foe[j]:
                ret_j = j
                j += 1
            else:
                ret_j = None
            yield ret_i, ret_j, cnt


    def run(i):
        print(i)
        if i[0] is not None:
            x_data_agent.append(x_agent[i[0]])
            y_data_agent.append(y_agent[i[0]])

        line1.set_data(x_data_agent, y_data_agent)

        if i[1] is not None:
            x_data_foe.append(x_foe[i[1]])
            y_data_foe.append(y_foe[i[1]])

        line2.set_data(x_data_foe, y_data_foe)

        x_data_biblio.append(i[2])
        y_data_biblio.append(vehicle_biblio[i[2]] / 50 * 100)
        line3.set_data(x_data_biblio, y_data_biblio)

        x_data_terminal.append(i[2])
        y_data_terminal.append(vehicle_terminal[i[2]] / 50 * 100)
        line4.set_data(x_data_terminal, y_data_terminal)

        x_data_multipiano.append(i[2])
        y_data_multipiano.append(vehicle_multipiano[i[2]] / 50 * 100)
        line5.set_data(x_data_multipiano, y_data_multipiano)

        if i[2] == x_agent[0]:
            ax1.annotate('starting\ncars',
                         xy=(x_agent[0], y_agent[0] + 30), xycoords='data',
                         xytext=(x_agent[0], y_agent[0] + 200),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='top')

        if i[2] == x_foe[-1] - 5:
            ax1.annotate('parked\ncars',
                         xy=(x_agent[-1] + 5, y_agent[-1]), xycoords='data',
                         xytext=(x_agent[-1] + (x_foe[-1] - x_agent[-1]) / 2, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')
            ax1.annotate(' ',
                         xy=(x_foe[-1] - 5, y_agent[-1]), xycoords='data',
                         xytext=(x_foe[-1] - ((x_foe[-1] - x_agent[-1]) / 2) + 5, y_agent[-1]),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')

        if i[2] == istant_1:
            ax1.axvline(x=istant_1, linestyle="--")
            ax1.annotate('Multipiano\nget empty', xy=(istant_1 - 1, 650), xycoords='data',
                         horizontalalignment='right', verticalalignment='center', color="tab:blue")

        if i[2] == istant_2:
            ax1.axvline(x=istant_2, linestyle="--")
            ax1.annotate('Terminal\nget empty', xy=(istant_2 + 5, 10), xycoords='data',
                         horizontalalignment='left', verticalalignment='center', color="tab:blue")

        return [line1, line2, line3, line4, line5]


    ani = animation.FuncAnimation(fig, run, data_gen, interval=200, blit=True, save_count=x_sup - x_inf + 1)
    ani.save("plot/gif/scenario_" + scena + ".gif", writer='pillow')

    # plt.show()
    fig.savefig("plot/scenario_" + scena + ".png")
    fig.savefig("plot/scenario_" + scena + ".pdf")


elif scena == "23":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, \
    vehicle_multipiano, _, _, _, _ = pickle.load(open("output_data/scenario_" + scena + ".p", "rb"))
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

    fig = plt.figure(figsize=(12, 6.72), dpi=200, tight_layout=True)
    gs = gridspec.GridSpec(3, 2)
    ax1 = fig.add_subplot(gs[:, 0])

    color1 = "tab:red"
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance Crowd Car (m)", color=color1)
    ax1.set(title="Distance traveled - " + name_scenario)

    x_data_agent, y_data_agent = [], []
    line1, = ax1.plot(x_data_agent, y_data_agent, color=color1)
    ax1.xaxis.grid()
    ax1.tick_params(axis="y", labelcolor=color1)
    ax1.set_ylim(-5 / 100 * max(y_agent), max(y_agent) + 5 / 100 * max(y_agent))
    x_min = min(x_agent[0], x_foe[0])
    x_max = max(x_agent[-1], x_foe[-1])
    x_inf = math.floor(x_min - 5 / 100 * (x_max - x_min))
    x_sup = math.ceil(x_max + 5 / 100 * (x_max - x_min))
    ax1.set_xlim(x_inf, x_sup)

    x_data_foe, y_data_foe = [], []
    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.set_ylabel("Distance Sumo Car (m)", color=color2)
    line2, = ax2.plot(x_data_foe, y_data_foe, color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)
    ax2.set_ylim(-5 / 100 * max(y_foe), max(y_foe) + 5 / 100 * max(y_foe))

    ax3 = fig.add_subplot(gs[0, 1])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 1])

    ax5.set_xlabel("Time (s)")
    ax3.set(title="Occupancy percentage")

    ax3.set_ylabel("Biblioteca(%)")
    x_data_biblio, y_data_biblio = [], []
    line3, = ax3.plot(x_data_biblio, y_data_biblio)
    ax3.xaxis.grid()
    ax3.yaxis.grid()
    ax3.tick_params(axis="y")
    ax3.set_ylim(-15, 115)
    ax3.set_xlim(x_inf, x_sup)

    ax4.set_ylabel("Terminal(%)")
    x_data_terminal, y_data_terminal = [], []
    line4, = ax4.plot(x_data_terminal, y_data_terminal)
    ax4.xaxis.grid()
    ax4.yaxis.grid()
    ax4.tick_params(axis="y")
    ax4.set_ylim(-15, 115)
    ax4.set_xlim(x_inf, x_sup)

    ax5.set_ylabel("Multipiano(%)")
    x_data_multipiano, y_data_multipiano = [], []
    line5, = ax5.plot(x_data_multipiano, y_data_multipiano)
    ax5.xaxis.grid()
    ax5.yaxis.grid()
    ax5.tick_params(axis="y")
    ax5.set_ylim(-15, 115)
    ax5.set_xlim(x_inf, x_sup)


    def data_gen():
        i = 0
        j = 0
        for cnt in range(x_inf, x_sup):
            if i < len(x_agent) and cnt == x_agent[i]:
                ret_i = i
                i += 1
            else:
                ret_i = None
            if j < len(x_foe) and cnt == x_foe[j]:
                ret_j = j
                j += 1
            else:
                ret_j = None
            yield ret_i, ret_j, cnt


    def run(i):
        print(i)
        if i[0] is not None:
            x_data_agent.append(x_agent[i[0]])
            y_data_agent.append(y_agent[i[0]])

        line1.set_data(x_data_agent, y_data_agent)

        if i[1] is not None:
            x_data_foe.append(x_foe[i[1]])
            y_data_foe.append(y_foe[i[1]])

        line2.set_data(x_data_foe, y_data_foe)

        x_data_biblio.append(i[2])
        y_data_biblio.append(vehicle_biblio[i[2]] / 50 * 100)
        line3.set_data(x_data_biblio, y_data_biblio)

        x_data_terminal.append(i[2])
        y_data_terminal.append(vehicle_terminal[i[2]] / 50 * 100)
        line4.set_data(x_data_terminal, y_data_terminal)

        x_data_multipiano.append(i[2])
        y_data_multipiano.append(vehicle_multipiano[i[2]] / 50 * 100)
        line5.set_data(x_data_multipiano, y_data_multipiano)

        if i[2] == x_agent[0]:
            ax1.annotate('starting\ncars',
                         xy=(x_agent[0], y_agent[0] + 30), xycoords='data',
                         xytext=(x_agent[0], y_agent[0] + 200),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='top')

        if i[2] == x_foe[-1] - 5:
            ax1.annotate('parked\ncars',
                         xy=(x_agent[-1] + 5, y_agent[-1] - 30), xycoords='data',
                         xytext=(x_agent[-1] + 5, y_agent[-1] - 200),
                         arrowprops=dict(facecolor='black', shrink=0.1),
                         horizontalalignment='center', verticalalignment='center')

        if i[2] == istant_1:
            ax1.axvline(x=istant_1, linestyle="--")
            ax1.annotate('Multipiano\nget empty', xy=(istant_1 - 1, 600), xycoords='data',
                         horizontalalignment='right', verticalalignment='center', color="tab:blue")
        if i[2] == istant_2:
            ax1.axvline(x=istant_2, linestyle="--")
            ax1.annotate('Terminal\nget empty', xy=(istant_2 + 1, 10), xycoords='data',
                         horizontalalignment='center', verticalalignment='center', color="tab:blue")
        if i[2] == istant_3:
            ax1.axvline(x=istant_3, linestyle="--")
            ax1.annotate('Biblioteca\nget empty', xy=(istant_3 + 1, 600), xycoords='data',
                         horizontalalignment='left', verticalalignment='center', color="tab:blue")

        return [line1, line2, line3, line4, line5]


    ani = animation.FuncAnimation(fig, run, data_gen, interval=200, blit=True, save_count=x_sup - x_inf + 1)
    ani.save("plot/gif/scenario_" + scena + ".gif", writer='pillow')

    # plt.show()
    fig.savefig("plot/scenario_" + scena + ".png")
    fig.savefig("plot/scenario_" + scena + ".pdf")


elif scena == "31":
    name_scenario, step_agents, distance_agents, step_foes, distance_foes, vehicle_biblio, vehicle_terminal, \
    vehicle_multipiano, means_agent, variance_agent, means_foe, variance_foe = pickle.load(
        open("output_data/scenario_" + scena + ".p", "rb"))

    n_agents = len(distance_agents)
    n_foes = len(distance_foes)

    x = [i for i in range(n_agents)]
    y_agent = [step[-1] - step[0] for step in step_agents]
    y_agent_mean = np.mean(y_agent)

    y_foe = [step[-1] - step[0] for step in step_foes]
    y_foe_mean = np.mean(y_foe)

    # travel time plot
    fig, (ax1, ax2) = plt.subplots(1, 2, sharey='all', figsize=(12, 6.72), dpi=200, tight_layout=True)
    ax1.set_xlabel("Crowd Car Number")
    ax1.set_ylabel("Travel Time (s)")
    ax1.set_title("Crowd Car")
    ax1.stem(x, y_agent, linefmt="tab:red", markerfmt="ro")
    ax1.axhline(y=y_agent_mean, color="tab:blue")
    ax1.xaxis.grid()
    ax1.yaxis.grid()
    ax1.text(0, y_agent_mean, 'Average travel time', ha='left', va='bottom', color='tab:blue')

    ax2.set_xlabel("Sumo Car Number")
    ax2.set_title("Sumo Car")
    ax2.stem(x, y_foe, linefmt="tab:green", markerfmt="go")
    ax2.axhline(y=y_foe_mean, color="tab:blue")
    ax2.xaxis.grid()
    ax2.yaxis.grid()
    ax2.text(0, y_foe_mean, 'Average travel time', ha='left', va='bottom', color='tab:blue')

    fig.suptitle("Travel Time - " + name_scenario, fontsize=16)
    # plt.show()
    fig.savefig("plot/scenario_" + scena + "_1_travel_time.png")
    fig.savefig("plot/scenario_" + scena + "_1_travel_time.pdf")
    # distance traveled plot
    y_agent_distance = [distance[-1] for distance in distance_agents]
    y_agent_mean_distance = np.mean(y_agent_distance)

    y_foe_distance = [distance[-1] for distance in distance_foes]
    y_foe_mean_distance = np.mean(y_foe_distance)

    fig, (ax1, ax2) = plt.subplots(1, 2, sharey='all', figsize=(12, 6.72), dpi=200, tight_layout=True)
    ax1.set_xlabel("Crowd Car Number")
    ax1.set_ylabel("Distance Traveled (m)")
    ax1.set_title("Crowd Car")
    ax1.stem(x, y_agent_distance, linefmt="tab:red", markerfmt="ro")
    ax1.axhline(y=y_agent_mean_distance, color="tab:blue")
    ax1.xaxis.grid()
    ax1.yaxis.grid()
    ax1.text(0, y_agent_mean_distance, 'Average distance', ha='left', va='bottom', color='tab:blue')

    ax2.set_xlabel("Sumo Car Number")
    ax2.set_title("Sumo Car")
    ax2.stem(x, y_foe_distance, linefmt="tab:green", markerfmt="go")
    ax2.axhline(y=y_foe_mean_distance, color="tab:blue")
    ax2.xaxis.grid()
    ax2.yaxis.grid()
    ax2.text(0, y_foe_mean_distance, 'Average distance', ha='left', va='bottom', color='tab:blue')

    fig.suptitle("Distance Traveled - " + name_scenario, fontsize=16)
    # plt.show()
    fig.savefig("plot/scenario_" + scena + "_2_distance_traveled.png")
    fig.savefig("plot/scenario_" + scena + "_2_distance_traveled.pdf")
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

    fig, ax = plt.subplots(figsize=(8, 6), dpi=200, tight_layout=True)
    rects1 = ax.bar(x - width / 2, set_agents, width, label='Crowd Cars', color="tab:red")
    rects2 = ax.bar(x + width / 2, set_foes, width, label='Sumo Cars', color="tab:green")

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel('Number of vehicles')
    ax.set_xlabel('Interval of travel time (s)')
    ax.set_title('Number of vehicles by type and travel time')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    ax.xaxis.grid()
    ax.yaxis.grid()

    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)

    fig.tight_layout()

    # plt.show()
    fig.savefig("plot/scenario_" + scena + "_3_bar_chart.png")
    fig.savefig("plot/scenario_" + scena + "_3_bar_chart.pdf")
