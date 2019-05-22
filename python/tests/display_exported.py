#! /usr/bin/python

import os
import sys
sys.path.append('../')
import numpy as np
import matplotlib.pyplot as plt

from data_prober import Metadata

from scipy.fftpack import fft

path = '/media/Nephelae-Data/data/M3-mission/data_export/data_set_0'

formatFilename = os.path.join(path, "dataformat.txt")
filename       = os.path.join(path, "all_metadata.txt")

mData = Metadata()
mData.parse_metadata(formatFilename, filename)

fig, axes = plt.subplots(2, 1, sharex=True, sharey=False)
axes[0].set_title("Speed (GPS vs odometry)")
axes[0].plot(mData.odometry_speed, '--o', label="Odometry")
axes[0].plot(mData.robot_to_world_speed, '--o', label="GPS")
axes[0].set_xlabel("Scan index")
axes[0].set_ylabel("Speed (m/s)")
axes[0].legend(loc="upper right")
axes[1].set_title("Number of points per scans")
axes[1].plot(mData.cloud_number_of_points, '--o', label="Nb points")
axes[1].set_xlabel("Scan index")
axes[1].set_ylabel("Number or points")
axes[1].legend(loc="upper right")

fig, axes = plt.subplots(1, 1, sharex=True, sharey=False)
axes.set_title("Speed (GPS vs odometry)")
axes.plot(mData.robot_to_world_pose_x, mData.robot_to_world_pose_y,  '--o', label="Robot position")
axes.set_xlabel("East (m)")
axes.set_ylabel("North (m)")
axes.legend(loc="upper right")
axes.set_aspect("equal")

fig, axes = plt.subplots(1, 1, sharex=True, sharey=False)
axes.plot(np.array(mData.robot_to_world_pose_sig_x) * 100.0, label="GPS sig X")
axes.plot(np.array(mData.robot_to_world_pose_sig_y) * 100.0, label="GPS sig Y")
axes.plot(np.array(mData.robot_to_world_pose_sig_z) * 100.0, label="GPS sig Z")
# axes.set_xlabel("East (m)")
axes.set_ylabel("Standard deviation (cm)")
axes.legend(loc="upper right")
# axes.set_aspect("equal")

# N = 10000
# h = np.zeros(N)
# h[0:101] = 1.0
# H = 20*np.log10(np.abs(fft(h)))
# H = H - np.amax(H)
# fig, axes = plt.subplots(1, 1, sharex=True, sharey=False)
# axes.plot(np.array([i / float(N) for i in range(N)]), H)


plt.show(block=False)

