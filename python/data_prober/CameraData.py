import os
import numpy as np
import io
from pyquaternion import Quaternion
from scipy.signal import medfilt

# copied from inscripts.core
from ruamel.yaml import YAML

from .Metadata import Metadata
from .InfuseTransform import InfuseTransform
from .DataCleaner import spike_detector

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
        self.calibrationFilename              = os.path.join(self.dataRootDir, "../", self.cameraName + "cam-calibration.yaml")

        # raw data file parsing
        self.dataLeft           = Metadata()
        self.dataRight          = Metadata()
        self.dataLeftIntegrity  = Metadata()
        self.dataRightIntegrity = Metadata()
        self.dataDisparity      = Metadata()
        self.dataCalibration    = None

        # Advanced data for display
        self.stereoDesync           = np.empty([0])
        self.stereoStamps           = np.empty([0])
        self.minTime                = -1
        self.disparityScore         = np.empty([0])
        self.disparityScoreFiltered = np.empty([0])
        self.leftIntegrity          = np.empty([0])
        self.rightIntegrity         = np.empty([0])
        self.totalScore             = np.empty([0])

        # other data
        self.imageNumber   = []
        self.leftToRobot   = []
        self.rightToLeft   = InfuseTransform()
        self.robotToWorld  = []

    def load(self):

        self.load_files()
        self.load_left_pose()
        self.load_robot_pose()
        self.compute_stereo_desync()
        self.compute_disparity_score()
        self.compute_integrity()
        self.compute_total_score()

    def load_files(self):

        self.dataLeft.parse_metadata(self.leftDataFormatFilename,   self.leftDataFilename)
        self.dataRight.parse_metadata(self.rightDataFormatFilename,   self.rightDataFilename)
        self.dataDisparity.parse_metadata(self.disparityDataFormatFilename, self.disparityDataFilename)
        self.dataLeftIntegrity.parse_metadata(self.leftIntegrityDataFormatFilename, self.leftIntegrityDataFilename)
        self.dataRightIntegrity.parse_metadata(self.rightIntegrityDataFormatFilename, self.rightIntegrityDataFilename)
        self.parse_calibration_file()

        self.check_file_consistency() # check if number of images is the same in each files

    def check_file_consistency(self):

        indexLeft               = np.array(self.dataLeft.index)
        indexRight              = np.array(self.dataRight.index)
        indexLeftIntegrity      = np.array(self.dataLeftIntegrity.index)
        indexRightIntegrity     = np.array(self.dataRightIntegrity.index)
        indexDisparity          = np.array(self.dataDisparity.index)

        try:
            if np.linalg.norm(indexLeft - indexRight) > 1e-6:
                raise Exception("Indexes inconsistent indexLeft/indexRight.")
            if np.linalg.norm(indexLeft - indexLeftIntegrity) > 1e-6:
                raise Exception("Indexes inconsistent indexLeft/indexLeftIntegrity.")
            if np.linalg.norm(indexLeft - indexRightIntegrity) > 1e-6:
                raise Exception("Indexes inconsistent indexLeft/indexRightIntegrity.")
            if np.linalg.norm(indexLeft - indexDisparity) > 1e-6:
                raise Exception("Indexes inconsistent indexLeft/indexDisparity.")
        except Exception as e:
            print("Data files are inconsistent for " + self.cameraName + " cam !")
            raise e

        self.imageNumber = self.dataLeft.index

    def compute_stereo_desync(self):

        self.stereoDesync = np.abs(self.dataRight.get_nparray('timestamp') - self.dataLeft.get_nparray('timestamp'))
        self.stereoStamps = self.dataLeft.get_nparray('timestamp')
        self.minTime = self.stereoStamps[0]

    def compute_disparity_score(self):

        self.disparityScore = 100 - self.dataDisparity.get_nparray('percentage_of_paired_pixels')
        self.disparityScore = self.disparityScore - np.amin(self.disparityScore)
        self.disparityScoreFiltered = spike_detector(self.disparityScore)

    def compute_integrity(self):
       
        self.leftIntegrity  = self.dataLeftIntegrity.get_nparray('score')
        self.leftIntegrity  = self.leftIntegrity - np.amin(self.leftIntegrity)
        self.rightIntegrity = self.dataRightIntegrity.get_nparray('score')
        self.rightIntegrity = self.rightIntegrity - np.amin(self.rightIntegrity)

    def compute_total_score(self):

        dataArray = np.vstack((self.stereoDesync, self.disparityScore, self.leftIntegrity, self.rightIntegrity))

        sig = np.zeros([4,4])
        sig[0,0] = 1.0 / np.dot(dataArray[0,:], dataArray[0,:])
        sig[1,1] = 1.0 / np.dot(dataArray[1,:], dataArray[1,:])
        sig[2,2] = 1.0 / np.dot(dataArray[2,:], dataArray[2,:])
        sig[3,3] = 1.0 / np.dot(dataArray[3,:], dataArray[3,:])

        self.totalScore = 1 - np.exp(-np.sum(np.multiply(dataArray, sig @ dataArray), axis=0))

    def parse_calibration_file(self):

        # yaml = YAML(typ='safe')
        # with open(self.calibrationFilename, 'r') as f:
        #     self.dataCalibration = yaml.load(f)

        # Had to parse by hand (issue with yaml version ?)
        def parse_yaml_data_line(line):
            items = line.split("data:")[1].split('[')[1].split(']')[0].split(',')
            return [float(item) for item in items]

        isInRotationBlock = False
        isInTranslationBlock = False
        for line in open(self.calibrationFilename, 'r'):

            if "rotation_matrix" in line:
                isInRotationBlock = True
                continue
            if isInRotationBlock and "data" in line:
                rotationData = parse_yaml_data_line(line)
                isInRotationBlock = False
                continue

            if "translation_coefficients" in line:
                isInTranslationBlock = True
                continue
            if isInTranslationBlock and "data" in line:
                translationData = parse_yaml_data_line(line)
                continue 

        translation    = np.array(translationData) 
        rotationMatrix = np.array([rotationData[0:3], rotationData[3:6], rotationData[6:10]])
        self.rightToLeft = InfuseTransform(translation, Quaternion(matrix=rotationMatrix))

    def load_left_pose(self):

        self.leftToRobot = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataLeft.pose_robot_sensor__x,
                                     self.dataLeft.pose_robot_sensor__y,
                                     self.dataLeft.pose_robot_sensor__z,
                                     self.dataLeft.pose_robot_sensor__qw,
                                     self.dataLeft.pose_robot_sensor__qx,
                                     self.dataLeft.pose_robot_sensor__qy,
                                     self.dataLeft.pose_robot_sensor__qz):
            self.leftToRobot.append(InfuseTransform(np.array([x,y,z]),
                                                    Quaternion([qw,qx,qy,qz])))
    def load_robot_pose(self):

        self.robotToWorld = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataLeft.pose_fixed_robot__x,
                                     self.dataLeft.pose_fixed_robot__y,
                                     self.dataLeft.pose_fixed_robot__z,
                                     self.dataLeft.pose_fixed_robot__qw,
                                     self.dataLeft.pose_fixed_robot__qx,
                                     self.dataLeft.pose_fixed_robot__qy,
                                     self.dataLeft.pose_fixed_robot__qz):
            self.robotToWorld.append(InfuseTransform(np.array([x,y,z]),
                                                     Quaternion([qw,qx,qy,qz])))

