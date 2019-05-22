import os
import numpy as np
import matplotlib.pyplot as plt
import io
from pyquaternion import Quaternion

from .Utils        import InfuseTransform
from .Utils        import spike_detector
from .Metadata     import Metadata
from .ExportedData import ExportedData

class ExportedVelodyneData(ExportedData):

    def __init__(self, dataRootDir, exportPath=""):
        super().__init__(os.path.join(dataRootDir, "export_plan.yaml"), exportPath)

        self.nbPoints = []
        self.bounds   = []

        self.dataPaths = [os.path.join(dataRootDir, "data"),
                          os.path.join(dataRootDir, "pngs")]
        self.dataExtensions = ['.pcd', '.png']
        self.dataExportSubPaths = ["data", "pngs"]

    def clean_data(self):

        super().clean_data()

        print("Clean data velodyne : ", self.dataToRemove)

        for index in reversed(self.dataToRemove):
            print("Popping nbPoint : ", self.nbPoints[index])
            self.nbPoints.pop(index)
            self.bounds.pop(index)

        self.dataToRemove = []

    def build_metadata_struct(self, interval):

        super().build_metadata_struct(interval)

        t0 = self.utcStamp[interval[0]]
        s = slice(interval[0], interval[-1] + 1)

        self.add_metadata('cloud_number_of_points', [int(n) for n in self.nbPoints[s]])
        self.add_metadata('cloud_min_x', [bbox[0] for bbox in self.bounds[s]])
        self.add_metadata('cloud_max_x', [bbox[1] for bbox in self.bounds[s]])
        self.add_metadata('cloud_min_y', [bbox[2] for bbox in self.bounds[s]])
        self.add_metadata('cloud_max_y', [bbox[3] for bbox in self.bounds[s]])
        self.add_metadata('cloud_min_z', [bbox[4] for bbox in self.bounds[s]])
        self.add_metadata('cloud_max_z', [bbox[5] for bbox in self.bounds[s]])


