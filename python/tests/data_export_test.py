#! /usr/bin/python3

import sys
sys.path.append('../')
import numpy as np
import matplotlib.pyplot as plt

from data_prober import DataCleaner2

# mana
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_04_13_28_09_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana/log_data_acquisition_2018_12_04_16_05_45_raw/raw_data/'


# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_11_29_14_49_58_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_11_29_15_45_57_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_11_29_16_31_41_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_02_14_07_32_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_03_12_19_48_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_03_14_14_25_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_03_15_07_37_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_04_12_10_06_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_04_13_28_09_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_04_16_05_45_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_05_13_49_09_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_05_15_59_36_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_07_14_42_33_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_07_16_37_11_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_08_13_35_33_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/mana2/log_data_acquisition_2018_12_08_18_36_28_raw/raw_data/'

# minnie 
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_02_05_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_04_13_07_22_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_05_14_12_30_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_07_14_27_38_raw/raw_data/'

# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_28_12_39_47_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_28_14_21_38_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_13_37_12_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_14_17_50_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_02_05_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_11_29_15_53_12_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_02_13_42_55_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_03_14_27_10_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_03_18_05_10_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_04_13_07_22_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_04_17_04_37_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_05_14_12_30_raw/raw_data/'
dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_05_17_13_59_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_07_12_40_13_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_07_14_27_38_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_07_16_34_33_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_08_12_27_01_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_08_15_54_46_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_08_16_43_34_raw/raw_data/'
# dataRootDir = '/media/Nephelae-Data/data/M3-mission/minnie/log_data_acquisition_2018_12_08_18_36_15_raw/raw_data/'


exportPath = '/media/Nephelae-Data/data/M3-mission/data_export'

dataCleaner = DataCleaner2(dataRootDir, exportPath)
dataCleaner.load()
dataCleaner.display(verbose=False)
# dataCleaner.export()
