#! /usr/bin/python3

from BrokenImageDetector import BrokenImageDetector

dataRootDir = '/local/infuse/data-test/minnie/log_data_acquisition_2018_12_05_14_12_30/raw_data/'
bid = BrokenImageDetector()

bid.detect(dataRootDir + 'front_cam/left/data/')
bid.write_score_file(dataRootDir + 'front_cam/left/')
bid.detect(dataRootDir + 'front_cam/right/data/')
bid.write_score_file(dataRootDir + 'front_cam/right/')

bid.detect(dataRootDir + 'rear_cam/left/data/')
bid.write_score_file(dataRootDir + 'rear_cam/left/')
bid.detect(dataRootDir + 'rear_cam/right/data/')
bid.write_score_file(dataRootDir + 'rear_cam/right/')

bid.detect(dataRootDir + 'nav_cam/left/data/')
bid.write_score_file(dataRootDir + 'nav_cam/left/')
bid.detect(dataRootDir + 'nav_cam/right/data/')
bid.write_score_file(dataRootDir + 'nav_cam/right/')

