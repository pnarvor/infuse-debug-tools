import os
import numpy as np
import io
from pyquaternion import Quaternion
from scipy.signal import medfilt

from .Metadata        import Metadata
from .InfuseTransform import InfuseTransform
from .RobotPoseData   import RobotPoseData
from .VelodyneData    import VelodyneData
from .CameraData      import CameraData

class DataCleaner2:
    
    def __init__(self, dataRootDir):

        self.dataRootDir = dataRootDir

        self.robotPoseData = RobotPoseData(dataRootDir)
        self.velodyneData  = VelodyneData(dataRootDir)
        self.navData       = CameraData(dataRootDir, "nav")
        self.frontData     = CameraData(dataRootDir, "front")
        self.rearData      = CameraData(dataRootDir, "rear")

    def load(self):

        self.robotPoseData.load()
        self.velodyneData.load()
        self.navData.load()

        self.velodyneData.compute_retagged_poses(self.robotPoseData)
        self.navData.compute_retagged_poses(self.robotPoseData)

    def display(self, verbose=False):

        self.robotPoseData.display(verbose)
        self.velodyneData.display(verbose)
        self.navData.display(verbose)
