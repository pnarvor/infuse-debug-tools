#! /usr/bin/python3

from Metadata import Metadata
from ImageSynchronizer import ImageSynchronizer
import numpy as np
import matplotlib.pyplot as plt
from  matplotlib.patches import Ellipse

# dataRootDir = '/local/infuse/data-test/minnie/log_data_acquisition_2018_12_05_14_12_30/raw_data/';
# dataRootDir = '/media/M3-data1/M3-mission/minnie/log_data_acquisition_2018_12_02_13_42_55/raw_data/'
# dataRootDir = '/local/infuse/data-test/mana/log_data_acquisition_2018_12_04_13_28_09/raw_data/';
dataRootDir = '/local/infuse/data-test/mana/log_data_acquisition_2018_12_04_12_10_06/raw_data/';

gps = Metadata()
gps.parse_metadata(dataRootDir + 'gps/gps_pose_info_dataformat.txt', dataRootDir + 'gps/gps_pose_info.txt')
x_gps = gps.get_nparray('x')
y_gps = gps.get_nparray('y')
time_gps = gps.get_nparray('child_time')
t0 = time_gps[0]

velodyne = Metadata()
velodyne.parse_metadata(dataRootDir + 'velodyne/dataformat.txt', dataRootDir + 'velodyne/all_metadata.txt')
cloud_time = velodyne.get_nparray('cloud_time')
robot_pose_time = velodyne.get_nparray('pose_fixed_robot__child_time')
sensor_pose_time = velodyne.get_nparray('pose_robot_sensor__child_time')

scale = [min([(cloud_time[0]  - t0) / 1000000.0, (robot_pose_time[0]  - t0) / 1000000.0, (sensor_pose_time[0]  - t0) / 1000000.0, (time_gps[0]  - t0) / 1000000.0]), 
         max([(cloud_time[-1] - t0) / 1000000.0, (robot_pose_time[-1] - t0) / 1000000.0, (sensor_pose_time[-1] - t0) / 1000000.0, (time_gps[-1] - t0) / 1000000.0])]

# fig, axes = plt.subplots(2,1, sharex=True, sharey=False)
fig, axes = plt.subplots(3,1, sharex=False, sharey=False)
axes[0].plot((cloud_time - t0) / 1000000.0, label="cloud_time")
axes[0].plot((robot_pose_time - t0) / 1000000.0, label="robot_pose_time")
axes[0].plot((sensor_pose_time - t0) / 1000000.0, label="sensor_pose_time")
axes[0].legend(loc="lower right")
axes[0].set_xlabel("index")
axes[0].set_ylabel("Time (s)")
axes[0].grid()

axes[1].plot((time_gps - t0) / 1000000.0, label="gps_time")
axes[1].legend(loc="lower right")
axes[1].set_xlabel("index")
axes[1].set_ylabel("Time (s)")
axes[1].grid()

axes[2].plot((cloud_time - t0) / 1000000.0, (cloud_time - robot_pose_time) / 1000.0, label="time difference cloud-robot_pose")
axes[2].plot((time_gps[:-1] - t0) / 1000000.0, (time_gps[1:] - time_gps[:-1]) / 1000.0, label="GPS time between successive poses")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Time (ms)")
# axes[2].set_xlim(scale)
axes[2].set_xlim([scale[0], 1880.0])
axes[2].grid()

fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
axes.plot((cloud_time - t0) / 1000000.0, (cloud_time - robot_pose_time) / 1000.0, label="time difference cloud-robot_pose")
axes.plot((time_gps[:-1] - t0) / 1000000.0, (time_gps[1:] - time_gps[:-1]) / 1000.0, label="GPS time between successive poses")
axes.legend(loc="upper right")
axes.set_xlabel("Mission time (s)")
axes.set_ylabel("Time (ms)")
# ax2].set_xlim(scale)
axes.set_xlim([scale[0], 1880.0])
axes.set_ylim([-100.0, 200.0])
axes.grid()

plt.show(block=False)




