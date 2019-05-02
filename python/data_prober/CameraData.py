import os
import numpy as np
import io
from pyquaternion import Quaternion
from scipy.signal import medfilt

from .Metadata import Metadata
from .InfuseTransform import InfuseTransform

class CameraData:

    def __init__(self, dataRootDir, camera="nav"):

        if not any([camera == name for name in ["nav", "front", "rear"]]):
            raise Exception("Error CameraData, invalid camera name [\"nav\",\"front\",\"rear\"] : " + camera)
        
        # Example of dataRootDir      : "/media/data/log_data_acquisition/raw_data"
        self.dataRootDir = dataRootDir
        self.cameraName = camera

        # raw file path
        self.leftDataFormatFilename           = os.path.join(self.dataRootDir, self.cameraName + "_cam/left/left_dataformat.txt")
        self.leftDataFilename                 = os.path.join(self.dataRootDir, self.cameraName + "_cam/left/left_all_metadata.txt")
        self.leftIntegrityDataFormatFilename  = os.path.join(self.dataRootDir, self.cameraName + "_cam/left/image_integrity_dataformat.txt")
        self.leftIntegrityDataFilename        = os.path.join(self.dataRootDir, self.cameraName + "_cam/left/image_integrity.txt")
        self.rightDataFormatFilename          = os.path.join(self.dataRootDir, self.cameraName + "_cam/right/right_dataformat.txt")
        self.rightDataFilename                = os.path.join(self.dataRootDir, self.cameraName + "_cam/right/right_all_metadata.txt")
        self.rightIntegrityDataFormatFilename = os.path.join(self.dataRootDir, self.cameraName + "_cam/right/image_integrity_dataformat.txt")
        self.rightIntegrityDataFilename       = os.path.join(self.dataRootDir, self.cameraName + "_cam/right/image_integrity.txt")
        self.disparityDataFormatFilename      = os.path.join(self.dataRootDir, self.cameraName + "_disparity/disparity_dataformat.txt")
        self.disparityDataFilename            = os.path.join(self.dataRootDir, self.cameraName + "_disparity/disparity_all_metadata.txt")
        self.calibrationFilename              = os.path.join(self.dataRootDir, "../../", self.cameraName + "-calibration.yaml")

        # raw data file parsing
        self.dataLeft           = Metadata()
        self.dataRight          = Metadata()
        self.dataLeftIntegrity  = Metadata()
        self.dataRightIntegrity = Metadata()
        self.dataDisparity      = Metadata()

        # Advanced data for display
        self.stereoDesync       = np.empty([0])
        self.stereoStamps       = np.empty([0])
        self.minTime            = -1
        self.disparityScore     = np.empty([0])
        self.leftIntegrity      = np.empty([0])
        self.rightIntegrity     = np.empty([0])
        self.totalScore         = np.empty([0])
        self.rightToLeft        = InfuseTransform()

    def load_files(self):

        self.dataLeft.parse_metadata(self.leftDataFormatFilename,   self.leftDataFilename)
        self.dataRight.parse_metadata(self.rightDataFormatFilename,   self.rightDataFilename)
        self.dataDisparity.parse_metadata(self.disparityDataFormatFilename, self.disparityDataFilename)
        self.dataLeftIntegrity.parse_metadata(self.leftIntegrityDataFormatFilename, self.leftIntegrityDataFilename)
        self.dataRightIntegrity.parse_metadata(self.rightIntegrityDataFormatFilename, self.rightIntegrityDataFilename)

    def compute_stereo_desync(self):

        self.stereoDesync = np.abs(self.dataRight.get_nparray('timestamp') - self.dataLeft.get_nparray('timestamp'))
        self.stereoStamps = self.dataLeft.get_nparray('timestamp')
        self.minTime = self.stereoStamps[0]

    def compute_disparity_score(self):

        self.disparityScore = 100 - self.dataDisparity.get_nparray('percentage_of_paired_pixels')
        self.disparityScore = self.disparityScore - np.amin(self.disparityScore)

    def compute_integrity(self):
       
        self.leftIntegrity  = self.dataLeftIntegrity.get_nparray('score')
        self.leftIntegrity  = self.leftIntegrity - np.amin(self.leftIntegrity)
        self.rightIntegrity = self.dataRightIntegrity.get_nparray('score')
        self.rightIntegrity = self.rightIntegrity - np.amin(self.rightIntegrity)

    def compute_total_score(self):

        dataArray = np.vstack((self.navStereoDesync, self.navDisparityScore, self.navLeftIntegrity, self.navRightIntegrity))

        sig = np.zeros([4,4])
        sig[0,0] = 1.0 / np.dot(dataArray[0,:], dataArray[0,:])
        sig[1,1] = 1.0 / np.dot(dataArray[1,:], dataArray[1,:])
        sig[2,2] = 1.0 / np.dot(dataArray[2,:], dataArray[2,:])
        sig[3,3] = 1.0 / np.dot(dataArray[3,:], dataArray[3,:])

        self.navTotalScore = 1 - np.exp(-np.sum(np.multiply(dataArray, sig @ dataArray), axis=0))
