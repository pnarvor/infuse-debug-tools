import os
import numpy as np
import matplotlib.pyplot as plt
import io
from pyquaternion import Quaternion
from scipy.signal import medfilt

# copied from inscripts.core
from ruamel.yaml import YAML

from .Metadata import Metadata
from .InfuseTransform import InfuseTransform
from .DataCleaner import spike_detector

class VelodyneData:

    def __init__(self, dataRootDir):

        # Example of dataRootDir      : "/media/data/log_data_acquisition/raw_data"
        self.dataRootDir = dataRootDir

        # raw file path
        self.dataFormatFilename  = os.path.join(self.dataRootDir, "velodyne/dataformat.txt")
        self.dataFilename        = os.path.join(self.dataRootDir, "velodyne/all_metadata.txt")

        # raw data file parsing
        self.dataVelodyne = Metadata()

        # data for display
        self.cloudTime = np.empty([0])
        self.poseTime  = np.empty([0])
        self.nbPoints  = np.empty([0])
        self.minTime   = -1

        # other data
        self.scanNumber    = []
        self.sensorToRobot = []
        self.robotToWorld  = []

    def load(self):

        self.load_files()
        self.load_cloud_data()
        self.load_sensor_pose()
        self.load_robot_pose()

    def load_files(self):

        self.dataVelodyne.parse_metadata(self.dataFormatFilename,   self.dataFilename)

    def load_cloud_data(self):

        self.cloudTime  = np.array(self.dataVelodyne.cloud_time)
        self.poseTime   = np.array(self.dataVelodyne.pose_fixed_robot__child_time)
        self.nbPoints   = np.array(self.dataVelodyne.nb_points)
        self.minTime    = self.cloudTime[0]

        self.scanNumber = self.dataVelodyne.index

    def load_sensor_pose(self):

        self.leftToRobot = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataVelodyne.pose_robot_sensor__x,
                                     self.dataVelodyne.pose_robot_sensor__y,
                                     self.dataVelodyne.pose_robot_sensor__z,
                                     self.dataVelodyne.pose_robot_sensor__qw,
                                     self.dataVelodyne.pose_robot_sensor__qx,
                                     self.dataVelodyne.pose_robot_sensor__qy,
                                     self.dataVelodyne.pose_robot_sensor__qz):
            self.leftToRobot.append(InfuseTransform(np.array([x,y,z]),
                                                    Quaternion([qw,qx,qy,qz])))
    def load_robot_pose(self):

        self.robotToWorld = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataVelodyne.pose_fixed_robot__x,
                                     self.dataVelodyne.pose_fixed_robot__y,
                                     self.dataVelodyne.pose_fixed_robot__z,
                                     self.dataVelodyne.pose_fixed_robot__qw,
                                     self.dataVelodyne.pose_fixed_robot__qx,
                                     self.dataVelodyne.pose_fixed_robot__qy,
                                     self.dataVelodyne.pose_fixed_robot__qz):
            self.robotToWorld.append(InfuseTransform(np.array([x,y,z]),
                                                     Quaternion([qw,qx,qy,qz])))

    def display(self):

        fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
        axes[0].plot((self.cloudTime - self.poseTime) / 1000.0, label="Desync cloud / pose")
        axes[0].legend(loc="upper right")
        axes[0].set_xlabel("Scan number")
        axes[0].set_ylabel("Desync (ms)")
        axes[0].grid()
        axes[1].plot(self.nbPoints, label="Desync cloud / pose")
        axes[1].legend(loc="upper right")
        axes[1].set_xlabel("Scan number")
        axes[1].set_ylabel("Number of points")
        axes[1].grid()
        axes[2].plot((self.cloudTime[1:] - self.cloudTime[:-1]) / 1000.0, label="Period")
        axes[2].legend(loc="upper right")
        axes[2].set_xlabel("Scan number")
        axes[2].set_ylabel("Period (ms)")
        axes[2].grid()

        plt.show(block=False)
