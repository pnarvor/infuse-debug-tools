import os
import numpy as np
import io
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from scipy.signal import medfilt
import copy as cp

import yaml

from .Utils               import InfuseTransform
from .Utils               import spike_detector
from .Utils               import add_twiny
from .Utils               import plot_highlighted
from .Metadata            import Metadata
from .ExportedCameraData import ExportedCameraData

class CameraData:

    def __init__(self, dataRootDir, camera="nav", exportPath=""):

        if not any([camera == name for name in ["nav", "front", "rear", "pano"]]):
            raise Exception("Error CameraData, invalid camera name [\"nav\",\"front\",\"rear\"] : " + camera)
        
        # Example of dataRootDir      : "/media/data/log_data_acquisition/raw_data"
        self.dataRootDir = dataRootDir
        self.cameraName  = camera
        self.filesLoaded = False

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
        self.exportPlanFilename               = os.path.join(self.dataRootDir, self.cameraName + "_cam/export_plan.yaml")
        self.exportPath                       = exportPath

        if self.cameraName == "pano":
            self.calibrationFilename              = os.path.join(self.dataRootDir, "../", "navcam-calibration.yaml")

        # raw data file parsing
        self.dataLeft           = Metadata()
        self.dataRight          = Metadata()
        self.dataLeftIntegrity  = Metadata()
        self.dataRightIntegrity = Metadata()
        self.dataDisparity      = Metadata()
        self.dataCalibration    = None

        # Data for display
        self.stereoDesync           = np.empty([0])
        self.stereoStamps           = np.empty([0])
        self.minTime                = -1
        self.disparityScore         = np.empty([0])
        self.disparityScoreFiltered = np.empty([0])
        self.leftIntegrity          = np.empty([0])
        self.rightIntegrity         = np.empty([0])
        self.totalScore             = np.empty([0])
        self.robotToWordTr          = np.empty([0])
        self.robotPoseRetaggedTr    = np.empty([0])
        self.odometryRetaggedTr  = np.empty([0])

        # other data
        self.imageNumber       = []
        self.leftToRobot       = []
        self.rightToLeft       = InfuseTransform()
        self.robotToWorld      = []
        self.robotPoseRetagged = []
        self.odometryRetagged  = []
        self.ltfToGtf          = None

        # Auto detect broken data
        self.suggestedBroken     = []
        self.desyncThreshold     = 50               # desync in ms
        self.pairedThreshold     = 15               # th in spike_detector(disparityScoreFiltered)
        self.maxGpsSigThreshold  = [5.0, 5.0, 10.0] # threshold of gps sigma in cm
        self.cameraSynchedStamps = []               # common stamps for stereo benches (not exported, internal use only)

    def load(self):

        try:
            self.load_files()
        except:
            print("No", self.cameraName + "_cam data") 
            return 

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
        self.filesLoaded = True

    def export(self):

        exporter = ExportedCameraData(self.dataRootDir, self.cameraName, self.exportPath)

        exporter.minTime     = self.minTime
        exporter.utcStamp    = list(cp.deepcopy(self.stereoStamps))
        exporter.dataIndex   = cp.deepcopy(self.imageNumber)
        exporter.ltfPose     = [p.tr        for p in self.robotPoseRetagged]
        exporter.ltfPoseTime = [p.stamp     for p in self.robotPoseRetagged]
        exporter.ltfCurvAbs  = [p.curveAbs  for p in self.robotPoseRetagged]
        exporter.gpsStddev   = [p.gpsStddev for p in self.robotPoseRetagged]
        exporter.odoAbsPose  = [p.tr        for p in self.odometryRetagged]
        exporter.odoPoseTime = [p.stamp     for p in self.odometryRetagged]
        exporter.odoCurvAbs  = [p.curveAbs  for p in self.odometryRetagged]
        exporter.sensorPose  = cp.deepcopy(self.leftToRobot)
        exporter.ltfToGtf    = self.ltfToGtf
        exporter.ltfSpeed    = [p.speed     for p in self.robotPoseRetagged]
        exporter.odoSpeed    = [p.speed     for p in self.odometryRetagged]

        exporter.parse_export_plan()
        exporter.clean_data()
        exporter.export()

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
        tmp = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataLeft.pose_fixed_robot__x,
                                     self.dataLeft.pose_fixed_robot__y,
                                     self.dataLeft.pose_fixed_robot__z,
                                     self.dataLeft.pose_fixed_robot__qw,
                                     self.dataLeft.pose_fixed_robot__qx,
                                     self.dataLeft.pose_fixed_robot__qy,
                                     self.dataLeft.pose_fixed_robot__qz):
            self.robotToWorld.append(InfuseTransform(np.array([x,y,z]),
                                                     Quaternion([qw,qx,qy,qz])))
            tmp.append([x,y,z])
        self.robotToWorldTr = np.array(tmp)

    def compute_retagged_poses(self, robotPoseData):

        if not self.filesLoaded:
            return

        self.robotPoseRetagged = robotPoseData.interpolate(self.dataLeft.timestamp)
        self.robotPoseRetaggedTr = np.array([[pose.tr.translation[0],
                                              pose.tr.translation[1],
                                              pose.tr.translation[2]] for pose in self.robotPoseRetagged])
        self.ltfToGtf = robotPoseData.localFrameTr

    def tag_odometry(self, robotPoseData):

        if not self.filesLoaded:
            return

        self.odometryRetagged = robotPoseData.interpolate_odometry(self.dataLeft.timestamp)
        self.odometryRetaggedTr = np.array([[pose.tr.translation[0],
                                             pose.tr.translation[1],
                                             pose.tr.translation[2]] for pose in self.odometryRetagged])
    def time_span(self):
        return [(self.stereoStamps[0] - self.minTime) / 1000000.0,
               (self.stereoStamps[-1] - self.minTime) / 1000000.0]

    def suggest_broken_data(self):
        
        self.suggestedBroken = []

        desync = abs(self.stereoDesync / 1000.0)
        self.suggestedBroken.extend(np.where(desync > self.desyncThreshold)[0].tolist())

        nbPoints = spike_detector(self.disparityScoreFiltered)
        self.suggestedBroken.extend(np.where(nbPoints > self.pairedThreshold)[0].tolist())

        gpsSigma = 100.0*np.array([p.gpsStddev for p in self.robotPoseRetagged])
        self.suggestedBroken.extend(np.where(gpsSigma[:,0] > self.maxGpsSigThreshold[0])[0].tolist())
        self.suggestedBroken.extend(np.where(gpsSigma[:,1] > self.maxGpsSigThreshold[1])[0].tolist())
        self.suggestedBroken.extend(np.where(gpsSigma[:,2] > self.maxGpsSigThreshold[2])[0].tolist())

        self.suggestedBroken.sort()
        print(self.cameraName + "_cam suggested images to remove :\n", self.suggestedBroken)

    def display(self, verbose=False, blocking=False):

        if not self.filesLoaded:
            return
 
        if verbose:
            fig, axes = plt.subplots(4,1, sharex=True, sharey=False)
            axes[0].set_title(self.cameraName + "_Data : Comparison original and posteriori pose")
            axes[0].plot(self.robotToWorldTr[:,0] - self.robotPoseRetaggedTr[:,0], '--o', label="Pose diff East", markeredgewidth=0.0)
            axes[0].legend(loc="upper right")
            axes[0].set_xlabel("Image number")
            axes[0].set_ylabel("Diff (m)")
            axes[0].grid()
            add_twiny(axes[0], self.time_span(), label="Mission time (s)")
            axes[1].plot(self.robotToWorldTr[:,1] - self.robotPoseRetaggedTr[:,1], '--o', label="Pose diff North", markeredgewidth=0.0)
            axes[1].legend(loc="upper right")
            axes[1].set_xlabel("Image number")
            axes[1].set_ylabel("Diff (m)")
            axes[1].grid()
            add_twiny(axes[1], self.time_span(), label="Mission time (s)")
            axes[2].plot(self.robotToWorldTr[:,2] - self.robotPoseRetaggedTr[:,2], '--o', label="Pose diff Elevation", markeredgewidth=0.0)
            axes[2].legend(loc="upper right")
            axes[2].set_xlabel("Image number")
            axes[2].set_ylabel("Diff (m)")
            axes[2].grid()
            add_twiny(axes[2], self.time_span(), label="Mission time (s)")
            axes[3].plot(np.linalg.norm(self.robotToWorldTr - self.robotPoseRetaggedTr, axis=1), '--o', label="Pose diff Norm", markeredgewidth=0.0)
            axes[3].legend(loc="upper right")
            axes[3].set_xlabel("Image number")
            axes[3].set_ylabel("Diff (m)")
            axes[3].grid()
            add_twiny(axes[3], self.time_span(), label="Mission time (s)")

       
            fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
            axes.set_title(self.cameraName + "_Data : Comparison original and posteriori pose")
            axes.plot(self.robotToWorldTr[:,0], self.robotToWorldTr[:,1], '--o', label="Pose tagged in Morocco", markeredgewidth=0.0)
            axes.plot(self.robotPoseRetaggedTr[:,0], self.robotPoseRetaggedTr[:,1], '--o', label="Pose retagged", markeredgewidth=0.0)
            axes.legend(loc="upper right")
            axes.set_xlabel("East (m)")
            axes.set_ylabel("North (m)")
            axes.set_aspect('equal')
            axes.grid()
            add_twiny(axes, self.time_span(), label="Mission time (s)")

        gpsSigma = 100.0*np.array([p.gpsStddev for p in self.robotPoseRetagged])

        fig, axes = plt.subplots(4,1, sharex=True, sharey=False)
        axes[0].set_title(self.cameraName + "_Data")
        # axes[0].plot(self.stereoDesync / 1000.0, label=self.cameraName+" Stereo desync")
        plot_highlighted(axes[0], self.stereoDesync / 1000.0, highlighted=self.suggestedBroken, label=self.cameraName+" Stereo desync")
        axes[0].legend(loc="upper right")
        axes[0].set_xlabel("Image index")
        axes[0].set_ylabel("Desync (ms)")
        axes[0].grid()
        add_twiny(axes[0], self.time_span(), label="Mission time (s)")
        axes[1].plot(self.disparityScore, label=self.cameraName+" disparity score")
        # axes[1].plot(self.disparityScoreFiltered, label=self.cameraName+" disparity score filtered")
        plot_highlighted(axes[1], self.disparityScoreFiltered, highlighted=self.suggestedBroken, label=self.cameraName+" disparity score filtered")
        axes[1].legend(loc="upper right")
        axes[1].set_xlabel("Image index")
        axes[1].set_ylabel("% unpaired pixels")
        axes[1].grid()
        add_twiny(axes[1], self.time_span(), label="Mission time (s)")
        axes[2].plot(self.leftIntegrity, label=self.cameraName+" left integrity")
        axes[2].plot(self.rightIntegrity, label=self.cameraName+" right integrity")
        axes[2].legend(loc="upper right")
        axes[2].set_xlabel("Image index")
        axes[2].set_ylabel("Image integrity (?)")
        axes[2].grid()
        add_twiny(axes[2], self.time_span(), label="Mission time (s)")
        # axes[3].plot(gpsSigma[:,0], label="Easting  sigma")
        # axes[3].plot(gpsSigma[:,1], label="Northing sigma")
        # axes[3].plot(gpsSigma[:,2], label="Height   sigma")
        plot_highlighted(axes[3], gpsSigma[:,0], highlighted=self.suggestedBroken, label="Easting  sigma")
        plot_highlighted(axes[3], gpsSigma[:,1], highlighted=self.suggestedBroken, label="Northing sigma")
        plot_highlighted(axes[3], gpsSigma[:,2], highlighted=self.suggestedBroken, label="Height   sigma")
        axes[3].legend(loc="upper right")
        axes[3].set_xlabel("Scan number")
        axes[3].set_ylabel("GPS sigma (cm)")
        axes[3].grid()
        add_twiny(axes[3], self.time_span(), label="Mission time (s)")

        plt.show(block=blocking)

