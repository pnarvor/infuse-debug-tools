#! /usr/bin/python3

from DataCleaner import DataCleaner
import numpy as np
import matplotlib.pyplot as plt


dataRootDir = '/local/infuse/data-test/minnie/log_data_acquisition_2018_12_05_14_12_30/raw_data/';

dataCleaner = DataCleaner(dataRootDir)
dataCleaner.compute_front_stereo_desync()
dataCleaner.compute_rear_stereo_desync()
dataCleaner.compute_nav_stereo_desync()
dataCleaner.compute_nav_disparity_score()
dataCleaner.compute_front_integrity()
dataCleaner.compute_rear_integrity()
dataCleaner.compute_nav_integrity()
dataCleaner.compute_nav_total_score()

t0 = dataCleaner.start_mission_time()

# fig, axes = plt.subplots(3,1, sharex=False, sharey=False)
# axes[0].plot(dataCleaner.frontStereoDesync / 1000.0, label="Front desync")
# axes[0].legend(loc="upper right")
# axes[0].set_xlabel("Image index")
# axes[0].set_ylabel("Desync time (ms)")
# axes[0].grid()
# axes[1].plot(dataCleaner.rearStereoDesync / 1000.0, label="Rear desync")
# axes[1].legend(loc="upper right")
# axes[1].set_xlabel("Image index")
# axes[1].set_ylabel("Desync time (ms)")
# axes[1].grid()
# axes[2].plot(dataCleaner.navStereoDesync / 1000.0,   label="Nav desync")
# axes[2].plot(dataCleaner.navDisparityScore, label="Nav disparity score")
# axes[2].legend(loc="upper right")
# axes[2].set_xlabel("Image index")
# axes[2].set_ylabel("Desync time (ms), % unpaired pixels")
# axes[2].grid()
# 
# fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
# axes[0].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontStereoDesync / 1000.0, label="Front desync")
# axes[0].legend(loc="upper right")
# axes[0].set_xlabel("Mission time (s)")
# axes[0].set_ylabel("Desync time (ms)")
# axes[0].grid()
# axes[1].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0,  dataCleaner.rearStereoDesync / 1000.0,  label="Rear desync")
# axes[1].legend(loc="upper right")
# axes[1].set_xlabel("Mission time (s)")
# axes[1].set_ylabel("Desync time (ms)")
# axes[1].grid()
# axes[2].plot((dataCleaner.navStereoStamps - t0) / 1000000.0,   dataCleaner.navStereoDesync / 1000.0,   label="Nav desync")
# axes[2].legend(loc="upper right")
# axes[2].set_xlabel("Mission time (s)")
# axes[2].set_ylabel("Desync time (ms)")
# axes[2].grid()
# 
# fig, axes = plt.subplots(3,1, sharex=False, sharey=False)
# axes[0].plot(dataCleaner.frontLeftIntegrity,  label="Front left integrity")
# axes[0].plot(dataCleaner.frontRightIntegrity, label="Front right integrity")
# axes[0].legend(loc="upper right")
# axes[0].set_xlabel("Image index")
# axes[0].set_ylabel("Integrity (?)")
# axes[0].grid()
# axes[1].plot(dataCleaner.rearLeftIntegrity,  label="Rear left integrity")
# axes[1].plot(dataCleaner.rearRightIntegrity, label="Rear right integrity")
# axes[1].legend(loc="upper right")
# axes[1].set_xlabel("Image index")
# axes[1].set_ylabel("Desync time (ms)")
# axes[1].set_ylabel("Integrity (?)")
# axes[1].grid()
# axes[2].plot(dataCleaner.navLeftIntegrity,  label="Nav left integrity")
# axes[2].plot(dataCleaner.navRightIntegrity, label="Nav right integrity")
# axes[2].legend(loc="upper right")
# axes[2].set_xlabel("Image index")
# axes[2].set_ylabel("Integrity (?)")
# axes[2].grid()
# 
# fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
# axes[0].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontLeftIntegrity,  label="Front left integrity")
# axes[0].plot((dataCleaner.frontStereoStamps - t0) / 1000000.0, dataCleaner.frontRightIntegrity, label="Front right integrity")
# axes[0].legend(loc="upper right")
# axes[0].set_xlabel("Mission time (s)")
# axes[0].set_ylabel("Integrity (?)")
# axes[0].grid()
# axes[1].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, dataCleaner.rearLeftIntegrity,  label="Rear left integrity")
# axes[1].plot((dataCleaner.rearStereoStamps - t0) / 1000000.0, dataCleaner.rearRightIntegrity, label="Rear right integrity")
# axes[1].legend(loc="upper right")
# axes[1].set_xlabel("Mission time (s)")
# axes[1].set_ylabel("Desync time (ms)")
# axes[1].set_ylabel("Integrity (?)")
# axes[1].grid()
# axes[2].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navLeftIntegrity,  label="Nav left integrity")
# axes[2].plot((dataCleaner.navStereoStamps - t0) / 1000000.0, dataCleaner.navRightIntegrity, label="Nav right integrity")
# axes[2].legend(loc="upper right")
# axes[2].set_xlabel("Mission time (s)")
# axes[2].set_ylabel("Integrity (?)")
# axes[2].grid()
# 
# fig, axes = plt.subplots(2,1, sharex=True, sharey=False)
# axes[0].plot(dataCleaner.navDisparityScore, label="Nav disparity score")
# axes[0].legend(loc="upper right")
# axes[0].set_xlabel("Image index")
# axes[0].set_ylabel("Disparity score")
# axes[0].grid()
# axes[1].plot(dataCleaner.navLeftIntegrity,  label="Nav left integrity")
# axes[1].plot(dataCleaner.navRightIntegrity, label="Nav right integrity")
# axes[1].legend(loc="upper right")
# axes[1].set_xlabel("Image index")
# axes[1].set_ylabel("Integrity (?)")
# axes[1].grid()

fig, axes = plt.subplots(4,1, sharex=True, sharey=False)
axes[0].plot(dataCleaner.navStereoDesync / 1000.0, label="Nav Stereo desync")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Image index")
axes[0].set_ylabel("Desync (ms)")
axes[0].grid()
axes[1].plot(dataCleaner.navDisparityScore, label="Nav disparity score")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("Image index")
axes[1].set_ylabel("% unpaired pixels")
axes[1].grid()
axes[2].plot(dataCleaner.navLeftIntegrity, label="Nav left integrity")
axes[2].plot(dataCleaner.navRightIntegrity, label="Nav right integrity")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("Image index")
axes[2].set_ylabel("Image integrity (?)")
axes[2].grid()
axes[3].plot(dataCleaner.navTotalScore, label="Nav total score")
axes[3].legend(loc="upper right")
axes[3].set_xlabel("Image index")
axes[3].set_ylabel("Total score")
axes[3].grid()

plt.show(block=False)




