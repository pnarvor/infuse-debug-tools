#! /usr/bin/python3

import sys
sys.path.append('../')
import numpy as np

from data_prober import CameraData


# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_02_05_raw/raw_data/'
dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_04_13_07_22_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_05_14_12_30_raw/raw_data/'

navData = CameraData(dataRootDir, "nav")
navData.load()
t0 = navData.minTime

navData.display()

