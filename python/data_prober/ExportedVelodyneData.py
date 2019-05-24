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

class ExportedVelodyneData(ExportedData):

    def __init__(self, velodyneData, dataToRemove):
        super().__init__(velodyneData.dataRootDir, dataToRemove)

        self.dataPaths = [os.path.join(self.dataRootDir, "velodyne/data"),
                          os.path.join(self.dataRootDir, "velodyne/pngs")]
        self.dataExtensions = ['.pcd', '.png']
        self.dataExportSubPaths = ["data", "pngs"]
        
        self.minTime     = velodyneData.minTime
        self.utcStamp    = list(cp.deepcopy(velodyneData.cloudTime))
        self.dataIndex   = cp.deepcopy(velodyneData.scanNumber)
        self.ltfPose     = [p.tr        for p in velodyneData.robotPoseRetagged]
        self.ltfPoseTime = [p.stamp     for p in velodyneData.robotPoseRetagged]
        self.ltfCurvAbs  = [p.curveAbs  for p in velodyneData.robotPoseRetagged]
        self.gpsStddev   = [p.gpsStddev for p in velodyneData.robotPoseRetagged]
        self.odoAbsPose  = [p.tr        for p in velodyneData.odometryRetagged]
        self.odoPoseTime = [p.stamp     for p in velodyneData.odometryRetagged]
        self.odoCurvAbs  = [p.curveAbs  for p in velodyneData.odometryRetagged]
        self.sensorPose  = cp.deepcopy(velodyneData.sensorToRobot)
        self.ltfToGtf    = velodyneData.ltfToGtf
        self.ltfSpeed    = [p.speed     for p in velodyneData.robotPoseRetagged]
        self.odoSpeed    = [p.speed     for p in velodyneData.odometryRetagged]
        self.nbPoints    = list(cp.deepcopy(velodyneData.nbPoints))
        self.bounds = [[xm,xM,ym,yM,zm,zM] for xm,xM,ym,yM,zm,zM in zip(
                                                    velodyneData.dataVelodyne.min_x,
                                                    velodyneData.dataVelodyne.max_x,
                                                    velodyneData.dataVelodyne.min_y,
                                                    velodyneData.dataVelodyne.max_y,
                                                    velodyneData.dataVelodyne.min_z,
                                                    velodyneData.dataVelodyne.max_z)]

    def export(self, interval, t0, outputPath):
        print("Exporting velodyne data")
        super().export(interval, t0, outputPath)

    def clean_data(self):

        super().clean_data()

        print("Cleanning velodyne data")

        for index in reversed(self.dataToRemove):
            self.nbPoints.pop(index)
            self.bounds.pop(index)

        self.dataToRemove = []

    def build_metadata_struct(self, interval, t0):

        if len(interval) == 0:
            raise Exception("Nothing to export : issue with export plan ?")

        super().build_metadata_struct(interval, t0)

        # t0 = self.utcStamp[interval[0]]
        # t0 = self.minTime
        # interval = self.time_to_index(interval, self.utcStamp)
        s = slice(interval[0], interval[-1] + 1)

        self.add_metadata('cloud_number_of_points', [int(n) for n in self.nbPoints[s]])
        self.add_metadata('cloud_min_x', [bbox[0] for bbox in self.bounds[s]])
        self.add_metadata('cloud_max_x', [bbox[1] for bbox in self.bounds[s]])
        self.add_metadata('cloud_min_y', [bbox[2] for bbox in self.bounds[s]])
        self.add_metadata('cloud_max_y', [bbox[3] for bbox in self.bounds[s]])
        self.add_metadata('cloud_min_z', [bbox[4] for bbox in self.bounds[s]])
        self.add_metadata('cloud_max_z', [bbox[5] for bbox in self.bounds[s]])


