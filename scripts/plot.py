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

filename = sys.argv[1]

# iterations,consensus,bounding,molecules
data = np.loadtxt(filename, delimiter=',', unpack=True)


plt.rc('grid', linestyle="--", color='grey')
fig, axs = plt.subplots(3)
fig.suptitle('Water')
axs[0].plot(data[0][2:], data[1][2:], label="Velocity Consensus", color='k')
axs[1].plot(data[0][2:], data[2][2:], label="Missing bounds", color='k')
axs[2].plot(data[0][2:], data[3][2:], label="Molecules", color='k')

# axs[0].set_title("Velocity Consensus")
axs[0].grid()
axs[0].set_xticklabels([])
axs[0].legend()

# axs[1].set_title("Missing bounds")
axs[1].grid()
axs[1].set_xticklabels([])
axs[1].legend()

# axs[2].set_title("Molecules")
axs[2].set_xlabel("Iterations")
axs[2].grid()
axs[2].legend()


plt.show()
