import os
import numpy as np
import io
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from scipy.signal import medfilt
import copy as cp

import yaml

from .Utils                import InfuseTransform
from .Utils                import spike_detector
from .Utils                import add_twiny
from .Utils                import plot_highlighted
from .Metadata             import Metadata
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
        self.ltfToGtf          = None

        # Auto detect broken data
        self.suggestedBroken    = []
        self.scanPoseDesyncThreshold = 150              # desync in ms
        self.pointsThreshold         = 0.2              # th in %/100 spike_detector(nbPoints)
        self.maxGpsSigThreshold      = [5.0, 5.0, 10.0] # threshold of gps sigma in cm

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
        exporter.minTime     = self.minTime
        exporter.utcStamp    = list(cp.deepcopy(self.cloudTime))
        exporter.dataIndex   = cp.deepcopy(self.scanNumber)
        exporter.ltfPose     = [p.tr        for p in self.robotPoseRetagged]
        exporter.ltfPoseTime = [p.stamp     for p in self.robotPoseRetagged]
        exporter.ltfCurvAbs  = [p.curveAbs  for p in self.robotPoseRetagged]
        exporter.gpsStddev   = [p.gpsStddev for p in self.robotPoseRetagged]
        exporter.odoAbsPose  = [p.tr        for p in self.odometryRetagged]
        exporter.odoPoseTime = [p.stamp     for p in self.odometryRetagged]
        exporter.odoCurvAbs  = [p.curveAbs  for p in self.odometryRetagged]
        exporter.sensorPose  = cp.deepcopy(self.sensorToRobot)
        exporter.ltfToGtf    = self.ltfToGtf
        exporter.ltfSpeed    = [p.speed     for p in self.robotPoseRetagged]
        exporter.odoSpeed    = [p.speed     for p in self.odometryRetagged]

        exporter.nbPoints    = list(cp.deepcopy(self.nbPoints))
        exporter.bounds = [[xm,xM,ym,yM,zm,zM] for xm,xM,ym,yM,zm,zM in zip(
                                                        self.dataVelodyne.min_x,
                                                        self.dataVelodyne.max_x,
                                                        self.dataVelodyne.min_y,
                                                        self.dataVelodyne.max_y,
                                                        self.dataVelodyne.min_z,
                                                        self.dataVelodyne.max_z)]

        exporter.parse_export_plan()
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
        self.ltfToGtf = robotPoseData.localFrameTr

    def tag_odometry(self, robotPoseData):

        if not self.filesLoaded:
            return

        self.odometryRetagged = robotPoseData.interpolate_odometry(self.dataVelodyne.cloud_time)
        self.odometryRetaggedTr = np.array([[pose.tr.translation[0],
                                             pose.tr.translation[1],
                                             pose.tr.translation[2]] for pose in self.odometryRetagged])
    def time_span(self):
        return [(self.cloudTime[0] - self.minTime) / 1000000.0,
               (self.cloudTime[-1] - self.minTime) / 1000000.0]

    def suggest_broken_data(self):
        
        self.suggestedBroken = []

        desync = abs((self.cloudTime - self.poseTime) / 1000.0)
        self.suggestedBroken.extend(np.where(desync > self.scanPoseDesyncThreshold)[0].tolist())

        nbPoints = spike_detector(self.nbPoints)
        self.suggestedBroken.extend(np.where(nbPoints > np.amax(nbPoints)*self.pointsThreshold)[0].tolist())

        # gpsSigma = 100.0*np.array([p.gpsStddev for p in self.robotPoseRetagged])
        # self.suggestedBroken.extend(np.where(gpsSigma[:,0] > self.maxGpsSigThreshold[0])[0].tolist())
        # self.suggestedBroken.extend(np.where(gpsSigma[:,1] > self.maxGpsSigThreshold[1])[0].tolist())
        # self.suggestedBroken.extend(np.where(gpsSigma[:,2] > self.maxGpsSigThreshold[2])[0].tolist())
        
        # sorting and removing dupplicates
        self.suggestedBroken = list(dict.fromkeys(self.suggestedBroken))
        self.suggestedBroken.sort()
        print("Velodyne suggested scans to remove :\n", self.suggestedBroken)

    def display(self, verbose=False, blocking=False):

        if not self.filesLoaded:
            return

        gpsSigma = 100.0*np.array([p.gpsStddev for p in self.robotPoseRetagged])

        fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
        axes[0].set_title("VelodyneData")
        # axes[0].plot((self.cloudTime - self.poseTime) / 1000.0, label="Desync cloud / pose")
        plot_highlighted(axes[0], (self.cloudTime - self.poseTime) / 1000.0, highlighted=self.suggestedBroken, label="Desync cloud / pose")
        axes[0].legend(loc="upper right")
        axes[0].set_xlabel("Scan number")
        axes[0].set_ylabel("Desync (ms)")
        axes[0].grid()
        add_twiny(axes[0], self.time_span(), label="Mission time (s)")
        axes[1].plot(self.nbPoints, label="Number of points / clouds")
        # axes[1].plot(spike_detector(self.nbPoints), label="Number of points / clouds")
        plot_highlighted(axes[1], spike_detector(self.nbPoints), highlighted=self.suggestedBroken, label="Number of points / clouds")
        axes[1].legend(loc="lower right")
        axes[1].set_xlabel("Scan number")
        axes[1].set_ylabel("Number of points")
        axes[1].grid()
        add_twiny(axes[1], self.time_span(), label="Mission time (s)")
        # axes[2].plot(gpsSigma[:,0], label="Easting  sigma")
        # axes[2].plot(gpsSigma[:,1], label="Northing sigma")
        # axes[2].plot(gpsSigma[:,2], label="Height   sigma")
        plot_highlighted(axes[2], gpsSigma[:,0], highlighted=self.suggestedBroken, label="Easting  sigma")
        plot_highlighted(axes[2], gpsSigma[:,1], highlighted=self.suggestedBroken, label="Northing sigma")
        plot_highlighted(axes[2], gpsSigma[:,2], highlighted=self.suggestedBroken, label="Height   sigma")
        axes[2].legend(loc="upper right")
        axes[2].set_xlabel("Scan number")
        axes[2].set_ylabel("GPS sigma (cm)")
        axes[2].grid()
        # axes[3].plot((self.cloudTime[1:] - self.cloudTime[:-1]) / 1000.0, label="Period")
        # axes[3].legend(loc="upper right")
        # axes[3].set_xlabel("Scan number")
        # axes[3].set_ylabel("Period (ms)")
        # axes[3].grid()
        add_twiny(axes[2], self.time_span(), label="Mission time (s)")

        plt.show(block=blocking)
