#! /usr/bin/python3

import sys
sys.path.append('../')
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

from data_prober import Metadata


# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_02_05_raw/raw_data/'
dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_04_13_07_22_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_05_14_12_30_raw/raw_data/'

gpsData = Metadata()
gpsData.parse_metadata(os.path.join(dataRootDir, 'gps/gps_pose_info_dataformat.txt'),
                       os.path.join(dataRootDir, 'gps/gps_pose_info.txt'))

gpsTime = np.array(gpsData.child_time)
gpsX = np.array(gpsData.x)
gpsY = np.array(gpsData.y)
gpsZ = np.array(gpsData.z)
gpsPos = np.array([gpsX, gpsY, gpsZ]).transpose()

interpolator = CubicSpline(gpsTime, gpsPos, axis=0)


