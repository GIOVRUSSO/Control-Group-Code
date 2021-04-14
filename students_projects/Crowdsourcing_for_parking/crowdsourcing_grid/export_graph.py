import pickle

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np

for r_i in range(0, 4):

    reward, route, behav = pickle.load(open("output_data/data_reward" + str(r_i) + ".p", "rb"))

    fig = plt.figure(figsize=(8, 7), dpi=200, tight_layout=True)
    gs = gridspec.GridSpec(3, 3)

    ax1 = fig.add_subplot(gs[2, 0:3])
    ax1.set_xlabel("Nodes")
    ax1.set_ylabel("Reward")
    ax1.bar(np.arange(0, len(reward), 1), reward)
    ax1.set_xticks(np.arange(0, len(behav[0]), 1))
    ax1.set_yticks(np.arange(min(reward), max(reward) + 0.2, 1))
    ax1.grid()

    ax3 = fig.add_subplot(gs[0:2, 0])
    ax3.set_xlabel("k")
    ax3.set_ylabel("$x_k$")
    ax3.stem(np.arange(0, len(route), 1), route)
    ax3.set_xticks(np.arange(0, len(behav) + 0.2, 1))
    ax3.set_yticks(np.arange(0, 24.2, 3))
    ax3.set_yticks(np.arange(0, 24.2, 1))
    ax3.grid()

    ax2 = fig.add_subplot(gs[0:2, 1:3], projection='3d')
    ax2.set_xlabel("X")
    ax2.set_ylabel("k")
    ax2.set_zlabel("$\pi_{k|k-1}$")
    ax2.set_zlim(0, 1)

    _y = np.arange(0, len(behav), 1)
    _x = np.arange(0, len(behav[0]), 1)
    ax2.set_xticks(np.arange(0, len(behav[0]), 3))
    ax2.set_xticks(np.arange(0, len(behav[0]), 1), minor=True)
    ax2.set_yticks(np.arange(0, len(behav) + 0.2, 1))
    ax2.set_zticks(np.arange(0, 1.2, 0.5))

    ax2.grid(which="major", alpha=0.6)
    ax2.grid(which="minor", alpha=0.3)

    _xx, _yy = np.meshgrid(_x, _y)
    x, y = _xx.ravel(), _yy.ravel()

    a = np.array([])
    for e in behav:
        a = np.concatenate((a, np.array(e)), axis=0)
    a[a == 0] = np.nan
    # ax2.set_aspect('auto')
    ax2.bar3d(x, y, 0, 1, 1, a, shade=True)

    fig.savefig("plot/reward_" + str(r_i) + ".png")
    fig.savefig("plot/reward_" + str(r_i) + ".pdf")
