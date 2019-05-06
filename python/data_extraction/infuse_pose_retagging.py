#! /usr/bin/python3

import sys
sys.path.append('../')
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt
import argparse

from data_prober import DataPoseTagger
from data_prober import Metadata

def spike_detector_filter(data):
   return np.abs(data - medfilt(data, kernel_size=3)) 

# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_04_13_28_09_raw/raw_data'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_08_13_35_33_raw/raw_data'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_08_18_36_28_raw/raw_data'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_07_16_37_11_raw/raw_data'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_02_14_07_32_raw/raw_data'
dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_05_14_12_30_raw/raw_data'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_04_13_07_22_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_02_05_raw/raw_data'


tokamak = Metadata()
tokamak.parse_metadata(os.path.join(dataRootDir, "tokamak/dataformat.txt"),
                       os.path.join(dataRootDir, "tokamak/tokamak.txt"))

poseTagger = DataPoseTagger(dataRootDir)
poseTagger.load()

gpsPoses = np.array([[pose.translation[0], pose.translation[1], pose.translation[2]] for pose in poseTagger.gpsTr])

odoStamps = np.array([pose.stamp for pose in poseTagger.odometryTr])
gpsStamps = np.array([pose.stamp for pose in poseTagger.gpsTr     ])

gpsSpeed  = 1000000.0*np.linalg.norm(gpsPoses[1:,0:1] - gpsPoses[:-1,0:1], axis=1) / (gpsStamps[1:] - gpsStamps[:-1])
gpsDeltaP = np.linalg.norm(gpsPoses[1:,0:1] - gpsPoses[:-1,0:1], axis=1)

newStamps = np.linspace(gpsStamps[0], gpsStamps[-1], len(gpsStamps))
newPoses = poseTagger.get_gps_interpolated(newStamps)
print(newPoses.shape)

fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
axes[0].plot((gpsStamps[1:] - gpsStamps[0]) / 1000000.0, gpsDeltaP, label="GPS delta")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Mission time (s)")
axes[0].set_ylabel("Speed (m/s)")
axes[0].grid()
axes[1].plot((gpsStamps[1:] - gpsStamps[0]) / 1000000.0, (gpsStamps[1:] - gpsStamps[:-1]) / 1000.0, label="GPS period")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Mission time (s)")
axes[1].set_ylabel("Period (ms)")
axes[1].grid()
axes[2].plot((gpsStamps[1:] - gpsStamps[0]) / 1000000.0, gpsSpeed, label="GPS speed")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Speed (m/s)")
axes[2].grid()

fig, axes = plt.subplots(1,1, sharex=True, sharey=False)
axes.plot((gpsStamps[1:] - odoStamps[0]) / 1000000.0, (gpsStamps[1:] - odoStamps[0]) / 1000.0, '--o', label="GPS period")
axes.legend(loc="upper right")
axes.set_xlabel("Mission time (s)")
axes.set_ylabel("Period (ms)")
axes.grid()

# fig, axes = plt.subplots(2,1, sharex=True, sharey=False)
# axes[0].plot((odoStamps - odoStamps[0]) / 1000000.0, gpsStamps - odoStamps, label="GPS/Odo delay")
# axes[0].legend(loc="upper right")
# axes[0].set_xlabel("Mission time (s)")
# axes[0].set_ylabel("Desync (ms)")
# axes[0].grid()
# axes[1].plot((odoStamps[1:] - odoStamps[0]) / 1000000.0, odoStamps[1:] - odoStamps[:-1], label="Odo period")
# axes[1].plot((odoStamps[1:] - odoStamps[0]) / 1000000.0, gpsStamps[1:] - gpsStamps[:-1], label="GPS period")
# axes[1].legend(loc="upper right")
# axes[1].set_xlabel("Mission time (s)")
# axes[1].set_ylabel("Period (ms)")
# axes[1].grid()

fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
# axes[0].plot((newStamps - gpsStamps[0]) / 1000000.0, newPoses[:,0], '--o', label="GPS int X")
axes[0].plot((gpsStamps - gpsStamps[0]) / 1000000.0, gpsPoses[:,0], '--o', label="GPS raw X")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Mission time (s)")
axes[0].set_ylabel("Period (ms)")
axes[0].grid()
# axes[1].plot((newStamps - gpsStamps[0]) / 1000000.0, newPoses[:,1], '--o', label="GPS int Y")
axes[1].plot((gpsStamps - gpsStamps[0]) / 1000000.0, gpsPoses[:,1], '--o', label="GPS raw Y")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Mission time (s)")
axes[1].set_ylabel("Period (ms)")
axes[1].grid()
# axes[2].plot((newStamps - gpsStamps[0]) / 1000000.0, newPoses[:,2], '--o', label="GPS int Z")
axes[2].plot((gpsStamps - gpsStamps[0]) / 1000000.0, gpsPoses[:,2], '--o', label="GPS raw Z")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Period (ms)")
axes[2].grid()

fig, axes = plt.subplots(1,1, sharex=True, sharey=False)
axes.plot(newPoses[:,0], newPoses[:,1], '--o', label="GPS int X")
axes.plot(gpsPoses[:,0], gpsPoses[:,1], '--o', label="GPS raw X")
axes.legend(loc="upper right")
axes.set_xlabel("East (m)")
axes.set_ylabel("North (m)")
axes.grid()
axes.set_aspect('equal')

plt.show(block=False)




