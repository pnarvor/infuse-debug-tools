#! /usr/bin/python3

import sys
import numpy as np
import matplotlib.pyplot as plt
from  matplotlib.patches import Ellipse
from scipy.signal import medfilt
from scipy.misc import imread
from scipy.ndimage.filters import convolve

from Metadata import Metadata
from ImageSynchronizer import ImageSynchronizer
from BrokenImageDetector import BrokenImageDetector

dataRootDir = '/local/infuse/data-test/minnie/log_data_acquisition_2018_12_05_14_12_30/raw_data/';
# dataRootDir = '/media/M3-data1/M3-mission/minnie/log_data_acquisition_2018_12_02_13_42_55/raw_data/'

detector = BrokenImageDetector()
scores = detector.detect(dataRootDir + 'nav_cam/left/data/')

plt.show(block=False)

# kernel = np.array([[-1.,-2.,-1.], \
#                    [ 0., 0., 0.], \
#                    [ 1., 2., 1.]])
# kernel = np.array([[ 1., 2., 1.], \
#                    [ 2., 3., 2.], \
#                    [ 1., 2., 1.]])

# kernel = np.array([[-1.], \
#                    [ 0.], \
#                    [ 1.]])

kernel = np.array([[-1.], \
                   [ 1.]])

img_left  = imread(dataRootDir + 'nav_cam/left/data/00491.pgm', mode='F')
img_right = imread(dataRootDir + 'nav_cam/right/data/00491.pgm', mode='F')

img_left =  img_left[:,:500]
img_right = img_right[:,:500]

# img_left_gradient  = img_left[1:,:] - img_left[:-1,:]
# img_right_gradient = img_right[1:,:] - img_right[:-1,:]

img_left_gradient  = convolve(img_left , kernel)
img_right_gradient = convolve(img_right, kernel)

left_gradient  = np.sum(np.absolute(img_left_gradient), axis=1)
right_gradient = np.sum(np.absolute(img_right_gradient), axis=1)

left_detection  = detector.edge_detector(img_left)
right_detection = detector.edge_detector(img_right)

stereo = Metadata()
stereo.parse_metadata(dataRootDir + 'stereo/nav_disparity/disparity_dataformat.txt', dataRootDir + 'stereo/nav_disparity/disparity_all_metadata.txt')
time_stereo_left  = stereo.get_nparray('left_timestamp')
time_stereo_right = stereo.get_nparray('right_timestamp')
paired_pixels     = stereo.get_nparray('percentage_of_paired_pixels')
t0 = time_stereo_left[0]

filtered_paired_pixels = medfilt(paired_pixels, kernel_size = 3)

#### # fig, axes = plt.subplots(2,1, sharex=True, sharey=False)
#### fig, axes = plt.subplots(2,1, sharex=False, sharey=False)
#### axes[0].plot((time_stereo_left - t0) / 1000000.0, (time_stereo_right - time_stereo_left) / 1000.0, label="desync left-right")
#### axes[0].plot((time_stereo_left - t0) / 1000000.0, 100.0 - paired_pixels, label="% non paired pixels")
#### axes[0].legend(loc="upper right")
#### axes[0].set_xlabel("Mission time")
#### axes[0].set_ylabel("Desync time (ms), % non-paired pixels (%)")
#### axes[0].grid()
#### 
#### axes[1].plot((time_stereo_right - time_stereo_left) / 1000.0, label="desync left-right")
#### axes[1].plot(100.0 - paired_pixels, label="% non paired pixels")
#### axes[1].legend(loc="upper right")
#### axes[1].set_xlabel("image index")
#### axes[1].set_ylabel("Desync time (ms), % non-paired pixels (%)")
#### axes[1].grid()
#### 
#### fig, axes = plt.subplots(2,1, sharex=True, sharey=False)
#### axes[0].plot(100.0 - paired_pixels, label="% non paired pixels")
#### axes[0].legend(loc="upper right")
#### axes[0].set_xlabel("Mission time")
#### axes[0].set_ylabel("% non-paired pixels (%)")
#### axes[0].grid()
#### 
#### axes[1].plot(filtered_paired_pixels - paired_pixels, label="non paired pixels detector")
#### axes[1].legend(loc="upper right")
#### axes[1].set_xlabel("Image index")
#### axes[1].set_ylabel("% non-paired pixels (%)")
#### axes[1].grid()
#### 
fig, axes = plt.subplots(2,1, sharex=False, sharey=False)
axes[0].plot(left_gradient,  label="left vertical gradient")
axes[0].plot(right_gradient, label="right vertical gradient")
# axes[0].legend(loc="upper right")
# axes[0].set_xlabel("Mission time")
# axes[0].set_ylabel("% non-paired pixels (%)")
axes[0].grid()

axes[1].plot(left_detection,  label="left vertical gradient")
axes[1].plot(right_detection, label="right vertical gradient")
# axes[1].legend(loc="upper right")
# axes[1].set_xlabel("Mission time")
# axes[1].set_ylabel("% non-paired pixels (%)")
axes[1].grid()

# fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
# plt.imshow(img_left, cmap='viridis')
# 
# fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
# plt.imshow(img_left_gradient, cmap='viridis')

fig, axes = plt.subplots(2,1, sharex=False, sharey=False)
axes[0].plot(scores)
axes[0].grid()
axes[1].plot(scores - medfilt(scores, kernel_size=3))
axes[1].grid()

plt.show(block=False)




