# Script to create plots of time measurements for evaluating crowdsourcing algorithm computation performances

import matplotlib.pyplot as plt
import numpy as np

def read_all_data():
    f = open('time_measures.txt','r')
    allconts = f.readlines()
    f.close()
    data_time = []
    for i in range(0,21):
        data_time.append([])
    data_ss = []
    for i in range(0,21):
        data_ss.append([])
    for l in allconts:
        ls = l.split(':')
        index = int(ls[1])
        data_time[index-1].append(float(ls[3]))
        data_ss[index-1].append(float(ls[2]))
    ret_t = []
    ret_ss = []
    for i in range(0,21):
        ret_t.append(np.mean(data_time[i]))
        ret_ss.append(np.mean(data_ss[i]))
    return ret_t,ret_ss


dats_time, dats_ss = read_all_data()
avg_array = np.array(dats_time)
plt.style.use('ggplot')
x = range(0,21)
fig, ax = plt.subplots(figsize=(10,5))
ax.plot(x,avg_array,color='red',label=str('average time').lower().replace('_',' '))
# plt.plot(array)
ax.set_title('Time horizon for backward recursion and time to compute')
ax.set_xlabel('Time horizon')
ax.set_ylabel('Average needed time [s]')
ax.legend(loc='best')
plt.show()

avg_array = np.array(dats_ss)
plt.style.use('ggplot')
x = range(0,21)
fig, ax = plt.subplots(figsize=(10,5))
ax.plot(x,avg_array,color='red',label=str('average state space length').lower().replace('_',' '))
# plt.plot(array)
ax.set_title('Time horizon for backward recursion and analyzed edges')
ax.set_xlabel('Time horizon')
ax.set_ylabel('Average amount of edges [s]')
ax.legend(loc='best')
plt.show()