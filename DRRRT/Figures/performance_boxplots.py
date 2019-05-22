import matplotlib.pyplot as plt
import numpy as np
import pickle

num_trials = 1000
names = ['barriers', 'hallway', 'narrow', 'split', 'maze']

rrt_results = {name: pickle.load(open("/home/ryan/PycharmProjects/DRRRT/Output/rrt_{}_{}.p"
                                      "".format(name, num_trials), "rb")) for name in names}

drrrt_results = {name: pickle.load(open("/home/ryan/PycharmProjects/DRRRT/Output/drrrt_{}_{}.p"
                                        "".format(name, num_trials), "rb")) for name in names}

rrt_clears = np.array([[x[3] for x in rrt_results[name] if x[1]] for name in names]).T
rrt_clears[4] = np.append(rrt_clears[4], 25000)
drrrt_clears = np.array([[x[3] for x in drrrt_results[name] if x[1]] for name in names]).T

for i in range(5):
    plt.figure(figsize=(3, 5))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')

    bp = plt.gca().boxplot(rrt_clears[i], positions=[- 0.2], widths=[0.3], sym='', manage_xticks=False)
    for element in ['boxes', 'whiskers', 'fliers', 'medians', 'caps']:
        plt.setp(bp[element], color="#D7191C")

    bp = plt.gca().boxplot(drrrt_clears[i], positions=[+ 0.2], widths=[0.3], sym='', manage_xticks=False)
    for element in ['boxes', 'whiskers', 'fliers', 'medians', 'caps']:
        plt.setp(bp[element], color="#2C7BB6")

    plt.plot([], c='#D7191C', label='RRT')
    plt.plot([], c='#2C7BB6', label='DRRRT')
    plt.legend()
    plt.title("Performance on ``{}''".format(names[i].capitalize()))
    plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    plt.ylabel("Number of Clear calls", fontsize=12)

    if names[i] == 'maze':
        plt.ylim([0, 1481.6])
    else:
        plt.ylim([0, plt.gca().get_ylim()[1]])
    plt.tight_layout()
    plt.savefig("{}_boxplot.pdf".format(names[i]))
