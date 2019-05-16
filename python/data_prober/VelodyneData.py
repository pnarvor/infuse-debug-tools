import os
import numpy as np
import matplotlib.pyplot as plt
import io
from pyquaternion import Quaternion
from scipy.signal import medfilt
import copy as cp

# copied from inscripts.core
import yaml

from .Metadata             import Metadata
from .InfuseTransform      import InfuseTransform
from .DataCleaner          import spike_detector
from .ExportedVelodyneData import ExportedVelodyneData

class VelodyneData:

    def __init__(self, dataRootDir, exportPath=""):

        # Example of dataRootDir      : "/media/data/log_data_acquisition/raw_data"
        self.dataRootDir = dataRootDir
        self.filesLoaded  = False

        # raw file path
        self.dataFormatFilename = os.path.join(self.dataRootDir, "velodyne/dataformat.txt")
        self.dataFilename       = os.path.join(self.dataRootDir, "velodyne/all_metadata.txt")
        self.exportPlanFilename = os.path.join(self.dataRootDir, "velodyne/export_plan.yaml")
        self.exportPath         = exportPath

        # raw data file parsing
        self.dataVelodyne = Metadata()

        # data for display
        self.cloudTime           = np.empty([0])
        self.poseTime            = np.empty([0])
        self.nbPoints            = np.empty([0])
        self.minTime             = -1
        self.robotToWorldTr      = np.empty([0])
        self.robotPoseRetaggedTr = np.empty([0])
        self.odometryRetaggedTr  = np.empty([0])

        # other data
        self.scanNumber        = []
        self.sensorToRobot     = []
        self.robotToWorld      = []
        self.robotPoseRetagged = []
        self.odometryRetagged  = []

    def load(self):

        try:
            self.load_files()
        except:
            print("No velodyne data")
            return
            
        self.load_cloud_data()
        self.load_sensor_pose()
        self.load_robot_pose()

    def load_files(self):

        self.dataVelodyne.parse_metadata(self.dataFormatFilename,   self.dataFilename)
        self.filesLoaded = True

    def export(self):

        exporter = ExportedVelodyneData(os.path.join(self.dataRootDir, "velodyne"), self.exportPath)
        exporter.utcStamp    = cp.deepcopy(self.cloudTime)
        exporter.dataIndex   = cp.deepcopy(self.scanNumber)
        exporter.ltfPose     = [p.tr        for p in self.robotPoseRetagged]
        exporter.ltfPoseTime = [p.stamp     for p in self.robotPoseRetagged]
        exporter.ltfCurvAbs  = [p.curveAbs  for p in self.robotPoseRetagged]
        exporter.gpsStddev   = [p.gpsStddev for p in self.robotPoseRetagged]
        exporter.odoAbsPose  = [p.tr        for p in self.odometryRetagged]
        exporter.odoPoseTime = [p.stamp     for p in self.odometryRetagged]
        exporter.odoCurvAbs  = [p.curveAbs  for p in self.odometryRetagged]
        exporter.sensorPose  = cp.deepcopy(self.sensorToRobot)

        exporter.nbPoints    = cp.deepcopy(self.nbPoints)
        exporter.bounds = [[xm,xM,ym,yM,zm,zM] for xm,xM,ym,yM,zm,zM in zip(
                                                        self.dataVelodyne.min_x,
                                                        self.dataVelodyne.max_x,
                                                        self.dataVelodyne.min_y,
                                                        self.dataVelodyne.max_y,
                                                        self.dataVelodyne.min_z,
                                                        self.dataVelodyne.max_z)]
        exporter.clean_data()
        exporter.export()
    

    def load_cloud_data(self):

        self.cloudTime = np.array(self.dataVelodyne.cloud_time)
        self.poseTime  = np.array(self.dataVelodyne.pose_fixed_robot__child_time)
        self.nbPoints  = np.array(self.dataVelodyne.nb_points)
        self.minTime   = self.cloudTime[0]

        self.scanNumber = self.dataVelodyne.index

    def load_sensor_pose(self):

        self.sensorToRobot = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataVelodyne.pose_robot_sensor__x,
                                     self.dataVelodyne.pose_robot_sensor__y,
                                     self.dataVelodyne.pose_robot_sensor__z,
                                     self.dataVelodyne.pose_robot_sensor__qw,
                                     self.dataVelodyne.pose_robot_sensor__qx,
                                     self.dataVelodyne.pose_robot_sensor__qy,
                                     self.dataVelodyne.pose_robot_sensor__qz):
            self.sensorToRobot.append(InfuseTransform(np.array([x,y,z]),
                                                      Quaternion([qw,qx,qy,qz])))

    def load_robot_pose(self):

        self.robotToWorld = []
        robotToWorldTr = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataVelodyne.pose_fixed_robot__x,
                                     self.dataVelodyne.pose_fixed_robot__y,
                                     self.dataVelodyne.pose_fixed_robot__z,
                                     self.dataVelodyne.pose_fixed_robot__qw,
                                     self.dataVelodyne.pose_fixed_robot__qx,
                                     self.dataVelodyne.pose_fixed_robot__qy,
                                     self.dataVelodyne.pose_fixed_robot__qz):
            self.robotToWorld.append(InfuseTransform(np.array([x,y,z]),
                                                     Quaternion([qw,qx,qy,qz])))
            robotToWorldTr.append([x,y,z])
        self.robotToWorldTr = np.array(robotToWorldTr)

    def compute_retagged_poses(self, robotPoseData):

        if not self.filesLoaded:
            return

        self.robotPoseRetagged = robotPoseData.interpolate(self.dataVelodyne.cloud_time)
        self.robotPoseRetaggedTr = np.array([[pose.tr.translation[0],
                                              pose.tr.translation[1],
                                              pose.tr.translation[2]] for pose in self.robotPoseRetagged])

    def tag_odometry(self, robotPoseData):

        if not self.filesLoaded:
            return

        self.odometryRetagged = robotPoseData.interpolate_odometry(self.dataVelodyne.cloud_time)
        self.odometryRetaggedTr = np.array([[pose.tr.translation[0],
                                             pose.tr.translation[1],
                                             pose.tr.translation[2]] for pose in self.odometryRetagged])

    def display(self, verbose=False, blocking=False):

        if not self.filesLoaded:
            return 

        fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
        axes[0].set_title("VelodyneData")
        axes[0].plot((self.cloudTime - self.poseTime) / 1000.0, label="Desync cloud / pose")
        axes[0].legend(loc="upper right")
        axes[0].set_xlabel("Scan number")
        axes[0].set_ylabel("Desync (ms)")
        axes[0].grid()
        axes[1].plot(self.nbPoints, label="Number of points / clouds")
        axes[1].legend(loc="upper right")
        axes[1].set_xlabel("Scan number")
        axes[1].set_ylabel("Number of points")
        axes[1].grid()
        axes[2].plot((self.cloudTime[1:] - self.cloudTime[:-1]) / 1000.0, label="Period")
        axes[2].legend(loc="upper right")
        axes[2].set_xlabel("Scan number")
        axes[2].set_ylabel("Period (ms)")
        axes[2].grid()

        plt.show(block=blocking)
