#! /usr/bin/python3

from Metadata import Metadata
from ImageSynchronizer import ImageSynchronizer
import numpy as np
import matplotlib.pyplot as plt
from  matplotlib.patches import Ellipse

# dataRootDir = '/local/infuse/data-test/minnie/log_data_acquisition_2018_12_05_14_12_30/raw_data/';
dataRootDir = '/media/M3-data1/M3-mission/minnie/log_data_acquisition_2018_12_02_13_42_55/raw_data/'

gps = Metadata()
gps.parse_metadata(dataRootDir + 'gps/gps_pose_info_dataformat.txt', dataRootDir + 'gps/gps_pose_info.txt')
x_gps = gps.get_nparray('x')
y_gps = gps.get_nparray('y')
time_gps = gps.get_nparray('child_time')

tokamak = Metadata()
tokamak.parse_metadata(dataRootDir + 'tokamak/dataformat.txt', dataRootDir + 'tokamak/tokamak.txt')
x_tokamak = tokamak.get_nparray('x')
y_tokamak = tokamak.get_nparray('y')
time_tokamak = tokamak.get_nparray('child_time')
t0 = time_tokamak[0]

nav_cam = Metadata()
nav_cam.parse_metadata(dataRootDir + 'nav_cam/pair_dataformat.txt', dataRootDir + 'nav_cam/pair_all_metadata.txt')
x_nav_cam = nav_cam.get_nparray('left__pose_fixed_robot__x')
y_nav_cam = nav_cam.get_nparray('left__pose_fixed_robot__y')
time_nav_cam = nav_cam.get_nparray('left__pose_fixed_robot__child_time')

fig, axes = plt.subplots(1,1, sharex=True, sharey=False)
axes.plot(x_gps - x_gps[0], y_gps - y_gps[0], label="GPS_pose")
axes.plot(x_tokamak - x_tokamak[0], y_tokamak - y_tokamak[0], label="tokamak_pose")
axes.plot(x_nav_cam - x_nav_cam[0], y_nav_cam - y_nav_cam[0], label="nav_cam_pose")
axes.legend(loc="upper right")
axes.grid()
plt.gca().set_aspect('equal', adjustable='box')

fig, axes = plt.subplots(2,1, sharex=True, sharey=False)
axes[0].plot((time_gps - t0) / 1000000.0, x_gps - x_gps[0], label="GPS_x")
axes[0].plot((time_tokamak - t0) / 1000000.0, x_tokamak - x_tokamak[0], label="tokamak_x")
axes[0].plot((time_nav_cam - t0) / 1000000.0, x_nav_cam - x_nav_cam[0], label="nav_cam_x")
axes[0].legend(loc="upper right")
axes[0].grid()

axes[1].plot((time_gps - t0) / 1000000.0, y_gps - y_gps[0], label="GPS_y")
axes[1].plot((time_tokamak - t0) / 1000000.0, y_tokamak - y_tokamak[0], label="tokamak_y")
axes[1].plot((time_nav_cam - t0) / 1000000.0, y_nav_cam - y_nav_cam[0], label="nav_cam_y")
axes[1].legend(loc="upper right")
axes[1].grid()

plt.show(block=False)




