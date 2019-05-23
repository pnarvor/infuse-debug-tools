import os
import numpy as np
import io
from pyquaternion import Quaternion
from scipy.signal import medfilt

from .Utils         import InfuseTransform
from .Metadata      import Metadata
from .RobotPoseData import RobotPoseData
from .VelodyneData  import VelodyneData
from .CameraData    import CameraData

class DataCleaner2:
    
    def __init__(self, dataRootDir, exportPath=""):

        self.dataRootDir = dataRootDir
        self.exportPath  = exportPath

        self.robotPoseData = RobotPoseData(dataRootDir)
        self.velodyneData  = VelodyneData(dataRootDir, exportPath)
        self.navData       = CameraData(dataRootDir, "nav"  , exportPath)
        self.frontData     = CameraData(dataRootDir, "front", exportPath)
        self.rearData      = CameraData(dataRootDir, "rear" , exportPath)

        # Global data
        self.minTime       = -1

    def load(self):

        self.robotPoseData.load()
        self.velodyneData.load()
        # self.navData.load()
        # self.frontData.load()
        # self.rearData.load()

        self.compute_mission_time()

        self.velodyneData.compute_retagged_poses(self.robotPoseData)
        self.velodyneData.tag_odometry(self.robotPoseData)
        self.velodyneData.suggest_broken_data()

        # self.navData.compute_retagged_poses(self.robotPoseData)
        # self.navData.tag_odometry(self.robotPoseData)
        # self.navData.suggest_broken_data()

        # self.frontData.compute_retagged_poses(self.robotPoseData)
        # self.frontData.tag_odometry(self.robotPoseData)
        # self.frontData.suggest_broken_data()

        # self.rearData.compute_retagged_poses(self.robotPoseData)
        # self.rearData.tag_odometry(self.robotPoseData)
        # self.rearData.suggest_broken_data()

    def export(self):

        self.velodyneData.export()
        # self.navData.export()
        # self.frontData.export()
        # self.rearData.export()

    def compute_mission_time(self):

        times = []
        if self.velodyneData.minTime > 0:
            times.append(self.velodyneData.minTime)
        if self.navData.minTime > 0:
            times.append(self.navData.minTime)
        if self.frontData.minTime > 0:
            times.append(self.frontData.minTime)
        if self.rearData.minTime > 0:
            times.append(self.rearData.minTime)
        self.minTime = min(times)

        self.velodyneData.minTime = self.minTime
        self.navData.minTime      = self.minTime
        self.frontData.minTime    = self.minTime
        self.rearData.minTime     = self.minTime

    def display(self, verbose=False, blocking=False):

        # self.robotPoseData.display(verbose, blocking)
        self.velodyneData.display(verbose, blocking)
        # self.navData.display(verbose, blocking)
        # self.frontData.display(verbose, blocking)
        # self.rearData.display(verbose, blocking)
