import os
import numpy as np
import matplotlib.pyplot as plt
import io
from pyquaternion import Quaternion
import copy as cp

from .Utils        import InfuseTransform
from .Utils        import spike_detector
from .Metadata     import Metadata
from .ExportedData import ExportedData

class ExportedCameraData(ExportedData):

    # def __init__(self, dataRootDir, cameraName, exportPath=""):
    #     super().__init__(os.path.join(dataRootDir, cameraName + "_cam/export_plan.yaml"), exportPath)

    #     self.cameraName = cameraName

    #     self.dataPaths = [os.path.join(dataRootDir, self.cameraName + "_cam/left/data"),
    #                       os.path.join(dataRootDir, self.cameraName + "_cam/right/data"),
    #                       os.path.join(dataRootDir, self.cameraName + "_disparity/rect_left"),
    #                       os.path.join(dataRootDir, self.cameraName + "_disparity/rect_right")]
    #     self.dataExtensions = ['.pgm', '.pgm', '.pgm', '.pgm']
    #     self.dataExportSubPaths = ["raw/left", "raw/right", "rectified/left", "rectified/right"]
    #     self.cameraSynchedStamps = [] # common stamps for stereo benches (not exported, internal use only)

    def __init__(self, cameraData, dataToRemove, exportPath):
        super().__init__(cameraData.dataRootDir, dataToRemove, exportPath)

        self.cameraName = cameraData.cameraName

        self.dataPaths = [os.path.join(self.dataRootDir, self.cameraName + "_cam/left/data"),
                          os.path.join(self.dataRootDir, self.cameraName + "_cam/right/data"),
                          os.path.join(self.dataRootDir, self.cameraName + "_disparity/rect_left"),
                          os.path.join(self.dataRootDir, self.cameraName + "_disparity/rect_right")]
        self.dataExtensions = ['.pgm', '.pgm', '.pgm', '.pgm']
        self.dataExportSubPaths = ["raw/left", "raw/right", "rectified/left", "rectified/right"]

        self.cameraSynchedStamps = [] # common stamps for stereo benches (not exported, internal use only)

        self.minTime     = cameraData.minTime
        self.utcStamp    = list(cp.deepcopy(cameraData.cloudTime))
        self.dataIndex   = cp.deepcopy(cameraData.scanNumber)
        self.ltfPose     = [p.tr        for p in cameraData.robotPoseRetagged]
        self.ltfPoseTime = [p.stamp     for p in cameraData.robotPoseRetagged]
        self.ltfCurvAbs  = [p.curveAbs  for p in cameraData.robotPoseRetagged]
        self.gpsStddev   = [p.gpsStddev for p in cameraData.robotPoseRetagged]
        self.odoAbsPose  = [p.tr        for p in cameraData.odometryRetagged]
        self.odoPoseTime = [p.stamp     for p in cameraData.odometryRetagged]
        self.odoCurvAbs  = [p.curveAbs  for p in cameraData.odometryRetagged]
        self.sensorPose  = cp.deepcopy(cameraData.sensorToRobot)
        self.ltfToGtf    = cameraData.ltfToGtf
        self.ltfSpeed    = [p.speed     for p in cameraData.robotPoseRetagged]
        self.odoSpeed    = [p.speed     for p in cameraData.odometryRetagged]
        self.cameraSynchedStamps = [] # common stamps for stereo benches (not exported, internal use only)

    def clean_data(self):

        super().clean_data()
        print("Clean data " + self.cameraName + "_cam : ", self.dataToRemove)
        self.dataToRemove = []

    def time_to_index(self, timeInterval, stamps):
        return super().time_to_index(timeInterval, self.cameraSynchedStamps)

    def build_metadata_struct(self, interval, t0):
        super().build_metadata_struct(interval, t0)

        # t0 = self.utcStamp[interval[0]]
        # s = slice(interval[0], interval[-1] + 1)


