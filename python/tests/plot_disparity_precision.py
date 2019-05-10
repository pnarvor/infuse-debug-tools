#! /usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from  matplotlib.patches import Ellipse

focalLength = 339.5
baseline    = 0.27
precision   = 1 / 16.0

# d = np.linspace(0.1, 10, 1000)
# depth = focalLength * baseline / d
depth = np.linspace(0.5, 150, 1000)
d     = focalLength * baseline / depth
err = focalLength * baseline * ( 1.0 / d - 1.0 / (d + precision))


fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
axes[0].plot(d, depth, label="desync left-right")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("disparity value (pixel)")
axes[0].set_ylabel("Depth (m)")
axes[0].grid()
axes[1].plot(d, err, label="desync left-right")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("disparity value (pixel)")
axes[1].set_ylabel("Depth err (m)")
axes[1].grid()
axes[2].plot(d, 100.0 * err / depth, label="desync left-right")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("disparity value (pixel)")
axes[2].set_ylabel("Relative depth err (%)")
axes[2].grid()

fig, axes = plt.subplots(1,1, sharex=True, sharey=False)
axes.plot(depth, err, label="desync left-right")
axes.legend(loc="upper right")
axes.set_xlabel("Depth (m)")
axes.set_ylabel("Depth error (m)")
axes.grid()

plt.show(block=False)



