#! /usr/bin/python3

import sys
sys.path.append('../')
import numpy as np
import matplotlib.pyplot as plt

from data_prober import RobotPoseData
from data_prober import VelodyneData

# mana
dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_04_13_28_09_raw/raw_data/'

# minnie 
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_02_05_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_04_13_07_22_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_05_14_12_30_raw/raw_data/'

robotPoseData = RobotPoseData(dataRootDir)
robotPoseData.load()
velodyneData = VelodyneData(dataRootDir)
velodyneData.load()

print("Velodyne cloud time : ", str(velodyneData.dataVelodyne.cloud_time[0]), "-", str(velodyneData.dataVelodyne.cloud_time[-1]))
velodynePoses  = robotPoseData.interpolate(velodyneData.dataVelodyne.cloud_time)
velodynePoseTr = np.array([[pose.tr.translation[0], pose.tr.translation[1], pose.tr.translation[2]] for pose in velodynePoses])

# GPS and odometry in LTF (more interesting)
fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
axes.plot(robotPoseData.robotLtfTr[:,0], robotPoseData.robotLtfTr[:,1], '--o', label="Robot LTF", markeredgewidth=0.0)
axes.plot(velodynePoseTr[:,0], velodynePoseTr[:,1], '--o', label="Scan poses", markeredgewidth=0.0)
axes.legend(loc="upper right")
axes.set_xlabel("East (m)")
axes.set_ylabel("North (m)")
axes.set_aspect('equal')
axes.grid()

plt.show(block=False)

# robotPoseData.display()
# velodyneData.display()

