#! /usr/bin/python3

import sys
sys.path.append('../')
import numpy as np
import matplotlib.pyplot as plt

from data_prober import DataCleaner

# Mana :
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_11_29_14_49_58_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_05_13_49_09_raw/raw_data/'

# Minnie :
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_02_05_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_04_13_07_22_raw/raw_data/'
dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_05_14_12_30_raw/raw_data/'

dataCleaner = DataCleaner(dataRootDir)

dataCleaner.compute_front_stereo_desync()
dataCleaner.compute_rear_stereo_desync()
dataCleaner.compute_nav_stereo_desync()
dataCleaner.compute_nav_disparity_score()
dataCleaner.compute_front_disparity_score()
dataCleaner.compute_rear_disparity_score()
dataCleaner.compute_front_integrity()
dataCleaner.compute_rear_integrity()
dataCleaner.compute_nav_integrity()
dataCleaner.compute_nav_position_diff()
dataCleaner.compute_gps()
dataCleaner.compute_odometry()
dataCleaner.compute_delta_odometry()
dataCleaner.compute_tokamak()
dataCleaner.compute_velodyne()

t0 = dataCleaner.start_mission_time()

fig, axes = plt.subplots(3,1, sharex=False, sharey=False)
axes[0].plot(dataCleaner.navStereoDesync / 1000.0,   label="Nav desync")
axes[0].plot(dataCleaner.navDisparityScore, label="Nav disparity score")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Image index")
axes[0].set_ylabel("Desync time (ms), % unpaired pixels")
axes[0].grid()
axes[1].plot(dataCleaner.frontStereoDesync / 1000.0, label="Front desync")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Image index")
axes[1].set_ylabel("Desync time (ms)")
axes[1].grid()
axes[2].plot(dataCleaner.rearStereoDesync / 1000.0, label="Rear desync")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Image index")
axes[2].set_ylabel("Desync time (ms)")
axes[2].grid()

fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
axes[0].plot((dataCleaner.navStereoStamps - t0) / 1000000.0,   dataCleaner.navStereoDesync / 1000.0,   label="Nav desync")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Mission time (s)")
axes[0].set_ylabel("Desync time (ms)")
axes[0].grid()
axes[1].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontStereoDesync / 1000.0, label="Front desync")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Mission time (s)")
axes[1].set_ylabel("Desync time (ms)")
axes[1].grid()
axes[2].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0,  dataCleaner.rearStereoDesync / 1000.0,  label="Rear desync")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Desync time (ms)")
axes[2].grid()

fig, axes = plt.subplots(3,1, sharex=False, sharey=False)
axes[0].plot(dataCleaner.navLeftIntegrity,  label="Nav left integrity")
axes[0].plot(dataCleaner.navRightIntegrity, label="Nav right integrity")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Image index")
axes[0].set_ylabel("Integrity (?)")
axes[0].grid()
axes[1].plot(dataCleaner.frontLeftIntegrity,  label="Front left integrity")
axes[1].plot(dataCleaner.frontRightIntegrity, label="Front right integrity")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Image index")
axes[1].set_ylabel("Integrity (?)")
axes[1].grid()
axes[2].plot(dataCleaner.rearLeftIntegrity,  label="Rear left integrity")
axes[2].plot(dataCleaner.rearRightIntegrity, label="Rear right integrity")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Image index")
axes[2].set_ylabel("Desync time (ms)")
axes[2].set_ylabel("Integrity (?)")
axes[2].grid()

fig, axes = plt.subplots(4,1, sharex=True, sharey=False)
axes[0].plot(dataCleaner.navDisparityScore, label="Nav % uncorrelated pixels")
axes[0].plot(DataCleaner.spike_detector_filter(dataCleaner.navDisparityScore), label="Nav % uncorrelated pixels filtered")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Image index")
axes[0].set_ylabel("% uncorrelated pixels")
axes[0].grid()
axes[1].plot(dataCleaner.navPositionDiff, label="Nav position diff")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Image index")
axes[1].set_ylabel("Panorama residual detector (m)")
axes[1].grid()
axes[2].plot(dataCleaner.frontDisparityScore, label="Front % uncorrelated pixels")
axes[2].plot(DataCleaner.spike_detector_filter(dataCleaner.frontDisparityScore), label="Front % uncorrelated pixels filtered")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Image index")
axes[2].set_ylabel("% uncorrelated pixels")
axes[2].grid()
axes[3].plot(dataCleaner.rearDisparityScore, label="Rear % uncorrelated pixels")
axes[3].plot(DataCleaner.spike_detector_filter(dataCleaner.rearDisparityScore), label="Rear % uncorrelated pixels filtered")
axes[3].legend(loc="upper right")
axes[3].set_xlabel("Image index")
axes[3].set_ylabel("% uncorrelated pixels")
axes[3].grid()

# NavCam
fig, axes = plt.subplots(5,1, sharex=True, sharey=False)
axes[0].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navDisparityScore, label="Nav % uncorrelated pixels")
axes[0].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, DataCleaner.spike_detector_filter(dataCleaner.navDisparityScore), label="Nav % uncorrelated pixels filtered")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Mission time (s)")
axes[0].set_ylabel("% uncorrelated pixels")
axes[0].grid()
axes[1].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navPositionDiff, label="Nav position diff")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Mission time (s)")
axes[1].set_ylabel("Panorama residual detector (m)")
axes[1].grid()
axes[2].plot((dataCleaner.navStereoStamps - t0) / 1000000.0,   dataCleaner.navStereoDesync / 1000.0,   label="Nav desync")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Desync time (ms)")
axes[2].grid()
axes[3].plot((dataCleaner.gpsTime[1:] - t0) / 1000000.0     , dataCleaner.gpsPeriod / 1000.0     , label="Gps period")
axes[3].plot((dataCleaner.odometryTime[1:] - t0) / 1000000.0, dataCleaner.odometryPeriod / 1000.0, label="Odometry period")
axes[3].plot((dataCleaner.tokamakTime[1:] - t0) / 1000000.0 , dataCleaner.tokamakPeriod / 1000.0 , label="Tokamak period")
axes[3].legend(loc="upper right")
axes[3].set_xlabel("Mission time (s)")
axes[3].set_ylabel("Period (ms)")
axes[3].grid()
axes[4].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navLeftIntegrity,  label="Nav left integrity")
axes[4].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navRightIntegrity, label="Nav right integrity")
axes[4].legend(loc="upper right")
axes[4].set_xlabel("Mission time (s)")
axes[4].set_ylabel("Integrity (?)")
axes[4].grid()

# FrontCam
fig, axes = plt.subplots(4,1, sharex=True, sharey=False)
axes[0].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontDisparityScore, label="Front % uncorrelated pixels")
axes[0].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, DataCleaner.spike_detector_filter(dataCleaner.frontDisparityScore), label="Front % uncorrelated pixels filtered")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Mission time (s)")
axes[0].set_ylabel("% uncorrelated pixels")
axes[0].grid()
axes[1].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0,   dataCleaner.frontStereoDesync / 1000.0,   label="Front desync")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Mission time (s)")
axes[1].set_ylabel("Desync time (ms)")
axes[1].grid()
axes[2].plot((dataCleaner.gpsTime[1:] - t0) / 1000000.0     , dataCleaner.gpsPeriod / 1000.0     , label="Gps period")
axes[2].plot((dataCleaner.odometryTime[1:] - t0) / 1000000.0, dataCleaner.odometryPeriod / 1000.0, label="Odometry period")
axes[2].plot((dataCleaner.tokamakTime[1:] - t0) / 1000000.0 , dataCleaner.tokamakPeriod / 1000.0 , label="Tokamak period")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Period (ms)")
axes[2].grid()
axes[3].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontLeftIntegrity,  label="Front left integrity")
axes[3].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontRightIntegrity, label="Front right integrity")
axes[3].legend(loc="upper right")
axes[3].set_xlabel("Mission time (s)")
axes[3].set_ylabel("Integrity (?)")
axes[3].grid()

# RearCam
fig, axes = plt.subplots(4,1, sharex=True, sharey=False)
axes[0].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, dataCleaner.rearDisparityScore, label="Rear % uncorrelated pixels")
axes[0].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, DataCleaner.spike_detector_filter(dataCleaner.rearDisparityScore), label="Rear % uncorrelated pixels filtered")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Mission time (s)")
axes[0].set_ylabel("% uncorrelated pixels")
axes[0].grid()
axes[1].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0,   dataCleaner.rearStereoDesync / 1000.0,   label="Rear desync")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Mission time (s)")
axes[1].set_ylabel("Desync time (ms)")
axes[1].grid()
axes[2].plot((dataCleaner.gpsTime[1:] - t0) / 1000000.0     , dataCleaner.gpsPeriod / 1000.0     , label="Gps period")
axes[2].plot((dataCleaner.odometryTime[1:] - t0) / 1000000.0, dataCleaner.odometryPeriod / 1000.0, label="Odometry period")
axes[2].plot((dataCleaner.tokamakTime[1:] - t0) / 1000000.0 , dataCleaner.tokamakPeriod / 1000.0 , label="Tokamak period")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Period (ms)")
axes[2].grid()
axes[3].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, dataCleaner.rearLeftIntegrity,  label="Rear left integrity")
axes[3].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, dataCleaner.rearRightIntegrity, label="Rear right integrity")
axes[3].legend(loc="upper right")
axes[3].set_xlabel("Mission time (s)")
axes[3].set_ylabel("Integrity (?)")
axes[3].grid()

fig, axes = plt.subplots(1,1, sharex=True, sharey=False)
axes.plot((dataCleaner.gpsTime[1:] - t0) / 1000000.0     , dataCleaner.gpsPeriod / 1000.0     , label="Gps period")
axes.plot((dataCleaner.odometryTime[1:] - t0) / 1000000.0, dataCleaner.odometryPeriod / 1000.0, label="Odometry period")
axes.plot((dataCleaner.tokamakTime[1:] - t0) / 1000000.0 , dataCleaner.tokamakPeriod / 1000.0 , label="Tokamak period")
axes.legend(loc="upper right")
axes.set_xlabel("Mission time (s)")
axes.set_ylabel("Period (ms)")
axes.grid()

fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
axes[0].plot((dataCleaner.velodyneCloudTime[1:] - t0) / 1000000.0, dataCleaner.velodynePeriod / 1000.0, label="Velodyne period")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Mission time (s)")
axes[0].set_ylabel("Period (ms)")
axes[0].grid()
axes[1].plot((dataCleaner.velodyneCloudTime - t0) / 1000000.0, np.abs(dataCleaner.velodyneDesync / 1000.0), label="Velodyne desync / associated pose")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Mission time (s)")
axes[1].set_ylabel("Desync (ms)")
axes[1].grid()
axes[2].plot((dataCleaner.velodyneCloudTime - t0) / 1000000.0, dataCleaner.velodyneNbPoints, label="Velodyne nb points")
axes[2].plot((dataCleaner.velodyneCloudTime - t0) / 1000000.0, DataCleaner.spike_detector_filter(dataCleaner.velodyneNbPoints), label="Velodyne nb points filtered")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Nb points / cloud")
axes[2].grid()

fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
axes[0].plot(dataCleaner.velodynePeriod / 1000.0, label="Velodyne period")
axes[0].legend(loc="upper right")
axes[2].set_xlabel("Cloud number")
axes[0].set_ylabel("Period (ms)")
axes[0].grid()
axes[1].plot(np.abs(dataCleaner.velodyneDesync / 1000.0), label="Velodyne desync / associated pose")
axes[1].legend(loc="upper right")
axes[2].set_xlabel("Cloud number")
axes[1].set_ylabel("Desync (ms)")
axes[1].grid()
axes[2].plot(dataCleaner.velodyneNbPoints, label="Velodyne nb points")
axes[2].plot(DataCleaner.spike_detector_filter(dataCleaner.velodyneNbPoints), label="Velodyne nb points filtered")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Cloud number")
axes[2].set_ylabel("Nb points / cloud")
axes[2].grid()

# All
fig, axes = plt.subplots(7,1, sharex=True, sharey=False)
# axes[0].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navDisparityScore, label="Nav % uncorrelated pixels")
axes[0].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, DataCleaner.spike_detector_filter(dataCleaner.navDisparityScore), label="Nav % uncorrelated pixels filtered")
# axes[0].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontDisparityScore, label="Front % uncorrelated pixels")
axes[0].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, DataCleaner.spike_detector_filter(dataCleaner.frontDisparityScore), label="Front % uncorrelated pixels filtered")
# axes[0].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, dataCleaner.rearDisparityScore, label="Rear % uncorrelated pixels")
axes[0].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, DataCleaner.spike_detector_filter(dataCleaner.rearDisparityScore), label="Rear % uncorrelated pixels filtered")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Mission time (s)")
axes[0].set_ylabel("% uncorrelated pixels")
axes[0].grid()
axes[1].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navPositionDiff, label="Nav position diff")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Mission time (s)")
axes[1].set_ylabel("Panorama residual detector (m)")
axes[1].grid()
axes[2].plot((dataCleaner.navStereoStamps - t0) / 1000000.0,   dataCleaner.navStereoDesync / 1000.0,   label="Nav desync")
axes[2].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0,   dataCleaner.frontStereoDesync / 1000.0,   label="Front desync")
axes[2].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0,   dataCleaner.rearStereoDesync / 1000.0,   label="Rear desync")
axes[2].plot((dataCleaner.velodyneCloudTime - t0) / 1000000.0, np.abs(dataCleaner.velodyneDesync / 1000.0), label="Velodyne desync / associated pose")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Mission time (s)")
axes[2].set_ylabel("Desync time (ms)")
axes[2].grid()
axes[3].plot((dataCleaner.gpsTime[1:] - t0) / 1000000.0     , dataCleaner.gpsPeriod / 1000.0     , label="Gps period")
axes[3].plot((dataCleaner.odometryTime[1:] - t0) / 1000000.0, dataCleaner.odometryPeriod / 1000.0, label="Odometry period")
axes[3].plot((dataCleaner.tokamakTime[1:] - t0) / 1000000.0 , dataCleaner.tokamakPeriod / 1000.0 , label="Tokamak period")
axes[3].legend(loc="upper right")
axes[3].set_xlabel("Mission time (s)")
axes[3].set_ylabel("Period (ms)")
axes[3].grid()
axes[4].plot((dataCleaner.velodyneCloudTime[1:] - t0) / 1000000.0, dataCleaner.velodynePeriod / 1000.0, label="Velodyne period")
axes[4].legend(loc="upper right")
axes[4].set_xlabel("Mission time (s)")
axes[4].set_ylabel("Period (ms)")
axes[4].grid()
# axes[5].plot((dataCleaner.velodyneCloudTime - t0) / 1000000.0, dataCleaner.velodyneNbPoints, label="Velodyne nb points")
axes[5].plot((dataCleaner.velodyneCloudTime - t0) / 1000000.0, DataCleaner.spike_detector_filter(dataCleaner.velodyneNbPoints), label="Velodyne nb points filtered")
axes[5].legend(loc="upper right")
axes[5].set_xlabel("Mission time (s)")
axes[5].set_ylabel("Nb points / cloud")
axes[5].grid()
axes[6].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navLeftIntegrity,  label="Nav left integrity")
axes[6].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navRightIntegrity, label="Nav right integrity")
axes[6].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontLeftIntegrity,  label="Front left integrity")
axes[6].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontRightIntegrity, label="Front right integrity")
axes[6].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, dataCleaner.rearLeftIntegrity,  label="Rear left integrity")
axes[6].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, dataCleaner.rearRightIntegrity, label="Rear right integrity")
axes[6].legend(loc="upper right")
axes[6].set_xlabel("Mission time (s)")
axes[6].set_ylabel("Integrity (?)")
axes[6].grid()

# Trajectories
fig, axes = plt.subplots(1,1, sharex=True, sharey=False)
axes.plot(dataCleaner.gpsX - dataCleaner.gpsX[0],
          dataCleaner.gpsY - dataCleaner.gpsY[0], label="GPS")
axes.plot(dataCleaner.odometryX - dataCleaner.odometryX[0],
          dataCleaner.odometryY - dataCleaner.odometryY[0], label="Odometry")
axes.plot(dataCleaner.deltaOdometryIntegratedX - dataCleaner.deltaOdometryIntegratedX[0],
          dataCleaner.deltaOdometryIntegratedY - dataCleaner.deltaOdometryIntegratedY[0], label="Delta odometry (integrated)")
axes.plot(dataCleaner.tokamakX - dataCleaner.tokamakX[0],
          dataCleaner.tokamakY - dataCleaner.tokamakY[0], label="Tokamak")
axes.legend(loc="upper right")
axes.set_xlabel("East (m)")
axes.set_ylabel("North (m)")
axes.grid()
axes.set_aspect('equal')

fig, axes = plt.subplots(2,1, sharex=True, sharey=False)
axes[0].plot(dataCleaner.deltaOdometryX, label="Delta odometry x")
axes[0].plot(dataCleaner.deltaOdometryY, label="Delta odometry y")
axes[0].plot(dataCleaner.deltaOdometryZ, label="Delta odometry z")
axes[0].legend(loc="upper right")
axes[0].grid()
axes[1].plot(dataCleaner.dataDeltaOdometry.qw, label="Delta odometry qw")
axes[1].plot(dataCleaner.dataDeltaOdometry.qx, label="Delta odometry qx")
axes[1].plot(dataCleaner.dataDeltaOdometry.qy, label="Delta odometry qy")
axes[1].plot(dataCleaner.dataDeltaOdometry.qz, label="Delta odometry qz")
axes[1].legend(loc="upper right")
axes[1].grid()

plt.show(block=False)




