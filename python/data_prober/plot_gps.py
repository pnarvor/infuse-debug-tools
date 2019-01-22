#! /usr/bin/python3

from Metadata import Metadata
from ImageSynchronizer import ImageSynchronizer
import numpy as np
import matplotlib.pyplot as plt
from  matplotlib.patches import Ellipse

dataRootDir = '/local/infuse/data-test/minnie/log_data_acquisition_2018_12_05_14_12_30/raw_data/';


gps = Metadata()
gps.parse_metadata(dataRootDir + 'gps/gps_pose_info_dataformat.txt', dataRootDir + 'gps/gps_pose_info.txt')
x_gps = gps.get_nparray('x')
y_gps = gps.get_nparray('y')

sigx_gps = gps.get_nparray('easting_sigma')
sigy_gps = gps.get_nparray('northing_sigma')

time_gps = gps.get_nparray('child_time')
t0 = time_gps[0]

N = len(gps.easting_sigma)

images = ImageSynchronizer(dataRootDir)
# Front cam
frontLeftStamps  = images.dataFrontLeft.get_nparray('timestamp')
frontRightStamps = images.dataFrontRight.get_nparray('timestamp')

fig1 = plt.figure()
plt.plot(x_gps, y_gps, label="robot position")
plt.legend(loc="upper right")
plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.gca().set_aspect('equal', adjustable='box')

fig2 = plt.figure()
plt.plot(time_gps / 1000000.0, sigx_gps, label="easting sigma")
plt.plot(time_gps / 1000000.0, sigy_gps, label="northing sigma")
plt.xlabel("Mission time (s)")
plt.ylabel("sigma (m?)")
plt.legend(loc="upper right")

#### # Plotting Ellipses for 5*gps_sigma(high computation cost) ####################################
#### ellipses = [Ellipse(xy=np.array([x_gps[n],y_gps[n]]), width=5*sigx_gps[n], height=5*sigy_gps[n], angle=0.0) for n in range(N)]
#### fig3 = plt.figure()
#### # fig3, axes = plt.subplots(subplot_kw={'aspect': 'equal'})
#### axes=fig3.add_subplot(111)
#### # axes.add_artist(Ellipse(xy=(1, 1), width=2, height=2, facecolor='g', edgecolor='k', alpha=.1))
#### plt.plot(x_gps, y_gps, label="robot position")
#### for e in ellipses:
####     # e.set_clip_box(axes.bbox)
####     e.set_alpha(1)
####     axes.add_artist(e)
#### plt.gca().set_aspect('equal', adjustable='box')

fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
axes[0].plot((time_gps[:-1] - t0) / 1000000.0, (time_gps[1:] - time_gps[:-1]) / 1000.0, label="GPS time difference between succesive poses")
axes[0].set_xlabel("Mission time (s)")
axes[0].set_ylabel("Time (ms)")
axes[0].legend(loc="upper right")
axes[0].grid()

dNorm = np.sqrt((x_gps[1:] - x_gps[:-1])**2 + (y_gps[1:] - y_gps[:-1])**2)
axes[1].plot((time_gps[:-1] - t0) / 1000000.0, dNorm, label="GPS distance between successive poses")
axes[1].set_xlabel("Mission time (s)")
axes[1].set_ylabel("Distance (m)")
axes[1].legend(loc="upper right")
axes[1].grid()

axes[2].plot((frontLeftStamps - t0) / 1000000.0, (frontRightStamps - frontLeftStamps) / 1000.0, label="Time difference right-left front images")
axes[2].plot((frontLeftStamps[:-1] - t0) / 1000000.0, (frontLeftStamps[1:] - frontLeftStamps[:-1]) / 1000.0, label="Time between successive left front images")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Time (ms)")
axes[2].legend(loc="upper right")
axes[2].grid()

plt.show(block=False)




