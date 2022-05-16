#!/usr/bin/env python3
from cProfile import label
import sys

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats


def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), scipy.stats.sem(a)
    h = se * scipy.stats.t.ppf((1 + confidence) / 2., n-1)
    return m, m-h, m+h

# dir = sys.argv[1]

dataX = []
dataY1 = []
dataY2 = []
dataY3 = []

num_files = 7

for i in range(0, num_files):
    # iterations,consensus,bounding,molecules
    X, Y1, Y2, Y3 = np.loadtxt("exp.{:03d}.log".format(i), delimiter=',', unpack=True)
    dataX.append(X)
    dataY1.append(Y1)
    dataY2.append(Y2)
    dataY3.append(Y3)

dataX = np.array(dataX)
dataY1 = np.array(dataY1)
dataY2 = np.array(dataY2)
dataY3 = np.array(dataY3)

Y1 = []
Y2 = []
Y3 = []

for i in range(0,dataY1.shape[1]):
    m, l, h = mean_confidence_interval(dataY1[:,i])
    Y1.append([m, l, h])
    m, l, h = mean_confidence_interval(dataY2[:,i])
    Y2.append([m, l, h])
    m, l, h = mean_confidence_interval(dataY3[:,i])
    Y3.append([m, l, h])        

Y1 = np.array(Y1)
Y2 = np.array(Y2)
Y3 = np.array(Y3)

start = 5

plt.rc('grid', linestyle="--", color='grey')
plt.rcParams.update({'font.size': 12})

fig, axs = plt.subplots(3)
# fig.suptitle('Water')
axs[0].plot(dataX[0, start:], Y1[start:,0], label="Average velocity error", color='k')
axs[0].fill_between(dataX[0, start:], Y1[start:,1], Y1[start:,2], color='r', alpha=.1)

axs[1].plot(dataX[0, start:], Y2[start:,0], label="Remaining bonds", color='k')
axs[1].fill_between(dataX[0, start:], Y2[start:,1], Y2[start:,2], color='r', alpha=.1)

axs[2].plot(dataX[0, start:], Y3[start:,0], label="Amount of molecules", color='k')
axs[2].fill_between(dataX[0, start:], Y3[start:,1], Y3[start:,2], color='r', alpha=.1)

# axs[0].set_title("Velocity Consensus")
axs[0].grid()
axs[0].set_xticklabels([])
axs[0].legend(fontsize=16)

# axs[1].set_title("Missing bounds")
axs[1].grid()
axs[1].set_xticklabels([])
axs[1].legend(fontsize=16)

# axs[2].set_title("Molecules")
axs[2].set_xlabel("Iterations", fontsize=16)
axs[2].grid()
axs[2].legend(fontsize=16)

plt.tight_layout()
plt.subplots_adjust(hspace = .08)


plt.show()
