#! /usr/bin/python3

import numpy as np
from pypcd import pypcd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

pcdFilename = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_02_05_raw/raw_data_test_pcd/nav_disparity/point_cloud_data/00000.pcd'

pc = pypcd.PointCloud.from_path(pcdFilename)
points = np.array([[p[0], p[1], p[2]] for p in pc.pc_data])

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter(points[0::100,0], points[0::100,1], points[0::100,2])
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
plt.show(block=False)




