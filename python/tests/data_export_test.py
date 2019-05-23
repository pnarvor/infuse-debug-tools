#! /usr/bin/python3

import sys
sys.path.append('../')
import numpy as np
import matplotlib.pyplot as plt

from data_prober import DataCleaner2

# mana
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_04_13_28_09_raw/raw_data/'
dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_04_16_05_45_raw/raw_data/'

# minnie 
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_02_05_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_04_13_07_22_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_05_14_12_30_raw/raw_data/'

exportPath = '/media/Nephelae-Data/data/M3-mission/data_export'

dataCleaner = DataCleaner2(dataRootDir, exportPath)
dataCleaner.load()
dataCleaner.display(verbose=False)
