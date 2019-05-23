import os
import numpy as np
import matplotlib.pyplot as plt
import io
from pyquaternion import Quaternion

from .Utils        import InfuseTransform
from .Utils        import spike_detector
from .Metadata     import Metadata
from .ExportedData import ExportedData

class ExportedCameraData(ExportedData):

    def __init__(self, dataRootDir, cameraName, exportPath=""):
        super().__init__(os.path.join(dataRootDir, cameraName + "_cam/export_plan.yaml"), exportPath)

        self.cameraName = cameraName

        self.dataPaths = [os.path.join(dataRootDir, self.cameraName + "_cam/left/data"),
                          os.path.join(dataRootDir, self.cameraName + "_cam/right/data"),
                          os.path.join(dataRootDir, self.cameraName + "_disparity/rect_left"),
                          os.path.join(dataRootDir, self.cameraName + "_disparity/rect_right")]
        self.dataExtensions = ['.pgm', '.pgm', '.pgm', '.pgm']
        self.dataExportSubPaths = ["raw/left", "raw/right", "rectified/left", "rectified/right"]
        self.cameraSynchedStamps = [] # common stamps for stereo benches (not exported, internal use only)

    def clean_data(self):

        super().clean_data()
        print("Clean data " + self.cameraName + "_cam : ", self.dataToRemove)
        self.dataToRemove = []

    def time_to_index(self, timeInterval, stamps):
        return super().time_to_index(timeInterval, self.cameraSynchedStamps)

    def build_metadata_struct(self, interval):
        super().build_metadata_struct(interval)

        # t0 = self.utcStamp[interval[0]]
        # s = slice(interval[0], interval[-1] + 1)


