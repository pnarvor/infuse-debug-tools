#! /usr/bin/python3

from Metadata import Metadata
from ImageSynchronizer import ImageSynchronizer
import numpy as np
import matplotlib.pyplot as plt
from  matplotlib.patches import Ellipse

dataRootDir = '/local/infuse/data-test/minnie/log_data_acquisition_2018_12_05_14_12_30/raw_data/';
# dataRootDir = '/media/M3-data1/M3-mission/minnie/log_data_acquisition_2018_12_02_13_42_55/raw_data/'

stereo = Metadata()
stereo.parse_metadata(dataRootDir + 'stereo/nav_disparity/disparity_dataformat.txt', dataRootDir + 'stereo/nav_disparity/disparity_all_metadata.txt')
time_stereo_left  = stereo.get_nparray('left_timestamp')
time_stereo_right = stereo.get_nparray('right_timestamp')
paired_pixels     = stereo.get_nparray('percentage_of_paired_pixels')
t0 = time_stereo_left[0]

# fig, axes = plt.subplots(2,1, sharex=True, sharey=False)
fig, axes = plt.subplots(2,1, sharex=False, sharey=False)
axes[0].plot((time_stereo_left - t0) / 1000000.0, (time_stereo_right - time_stereo_left) / 1000.0, label="desync left-right")
axes[0].plot((time_stereo_left - t0) / 1000000.0, 100.0 - paired_pixels, label="% non paired pixels")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("Mission time")
axes[0].set_ylabel("Desync time (ms), % non-paired pixels (%)")
axes[0].grid()

axes[1].plot((time_stereo_right - time_stereo_left) / 1000.0, label="desync left-right")
axes[1].plot(100.0 - paired_pixels, label="% non paired pixels")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("image index")
axes[1].set_ylabel("Desync time (ms), % non-paired pixels (%)")
axes[1].grid()

plt.show(block=False)




