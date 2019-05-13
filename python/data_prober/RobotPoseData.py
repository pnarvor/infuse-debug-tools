import os
import numpy as np
import matplotlib.pyplot as plt
import io
from pyquaternion import Quaternion
import xml.etree.ElementTree as XmlTree
from scipy.signal import medfilt
# from scipy.interpolate import NearestNDInterpolator
from scipy.interpolate import interp1d


from .Metadata import Metadata
from .InfuseTransform import InfuseTransform
from .DataCleaner import spike_detector

def find_urdf(filenameHint):
    [path, hint] = os.path.split(filenameHint)
    for f in os.listdir(path): 
        if os.path.isfile(os.path.join(path, f)):
            if hint in f and os.path.splitext(f)[1] == ".urdf":
                return os.path.join(path, f)

def parse_urdf_joint(jointNode):

    transformNode = None
    for child in jointNode:
        if child.tag == 'origin':
            transformNode = child
    if transformNode is None:
        raise Exception("Error parsing", self.gpsUrdfFilename, ". File corrupted ?")

    t = transformNode.attrib['xyz'].split()

    return InfuseTransform(np.array([float(t[0]), float(t[1]), float(t[2])]))

class RobotPoseData:

    def __init__(self, dataRootDir):

        # Example of dataRootDir      : "/media/data/log_data_acquisition/raw_data"
        self.dataRootDir = dataRootDir

        # raw file path
        self.gpsDataFormatFilename      = os.path.join(self.dataRootDir, "gps/gps_pose_info_dataformat.txt")
        self.gpsDataFilename            = os.path.join(self.dataRootDir, "gps/gps_pose_info.txt")
        self.odometryDataFormatFilename = os.path.join(self.dataRootDir, "odometry/dataformat.txt")
        self.odometryDataFilename       = os.path.join(self.dataRootDir, "odometry/odometry.txt")
        self.tokamakDataFormatFilename  = os.path.join(self.dataRootDir, "tokamak/dataformat.txt")
        self.tokamakDataFilename        = os.path.join(self.dataRootDir, "tokamak/tokamak.txt")
        self.gpsUrdfFilename            = find_urdf(os.path.join(self.dataRootDir, "../tokamak_urdf_internal_"))
        self.localFramesUrdfFilename    = find_urdf(os.path.join(self.dataRootDir, "../fixed_frames_"))
        self.headingFilename            = os.path.join(self.dataRootDir, "../test_parking.txt")

        # raw data file parsing
        self.dataGps       = Metadata()
        self.dataOdometry  = Metadata()
        self.dataTokamak   = Metadata()
        self.localFrameTr  = None # LocalTerrainFrame (LTF) to GlobalTerrainFrame (GTF)
        self.gpsInternalTr = None # GPS antenna position in robot (relative to RobotBodyFrame, RBF)
        self.odoFrameToGTF = None # OdometryFrame (frame in which RMP export its data) to GTF

        # non-raw data
        self.gpsPoses         = []
        self.gpsLtfPoses      = []
        self.odometryPoses    = []
        self.odometryLtfPoses = []
        self.tokamakPoses     = []

        # data for display
        self.gpsTr         = np.empty([0])
        self.gpsLtfTr      = np.empty([0])
        self.odometryTr    = np.empty([0])
        self.odometryLtfTr = np.empty([0])
        self.tokamakTr     = np.empty([0])

    def load(self):

        self.load_files()
        self.load_gps_poses()
        self.load_odometry_poses()
        self.load_tokamak_poses()
        self.compute_gps_ltf()
        self.compute_odometry_ltf()
        self.compute_robot_pose_ltf()

    def load_files(self):

        self.parse_internal_urdf()
        self.parse_external_urdf()
        self.parse_heading_file()
        self.dataGps.parse_metadata(self.gpsDataFormatFilename, self.gpsDataFilename)
        self.dataOdometry.parse_metadata(self.odometryDataFormatFilename, self.odometryDataFilename)
        self.dataTokamak.parse_metadata(self.tokamakDataFormatFilename, self.tokamakDataFilename)

    def parse_internal_urdf(self):

        root = XmlTree.parse(self.gpsUrdfFilename).getroot()
        for child in root:
            if child.attrib['name'] == "Rover2GPS":
                self.gpsInternalTr = parse_urdf_joint(child)
        if self.gpsInternalTr is None:
            raise Exception("Error parsing", self.gpsUrdfFilename, ". File corrupted ?")

    def parse_external_urdf(self):

        root = XmlTree.parse(self.localFramesUrdfFilename).getroot()
        siteFrameTr  = None
        localFrameTr = None
        for child in root:
            if child.attrib['name'] == "SiteFrameToGlobalTerrainFrame":
                siteFrameTr  = parse_urdf_joint(child)
            if child.attrib['name'] == "LocalTerrainFrameToSiteFrame":
                localFrameTr = parse_urdf_joint(child)
        if siteFrameTr is None or localFrameTr is None:
            raise Exception("Error parsing", self.gpsUrdfFilename, ". File corrupted ?")

        self.localFrameTr = siteFrameTr * localFrameTr

    def parse_heading_file(self):

        try:
            headingFile = open(self.headingFilename, "r")
        except:
            raise Exception("No heading file found at : ", self.headingFilename)

        for line in headingFile:
            if "Position given in set heading: " in line:
                nums = line.split("Position given in set heading: ")[1].split(" ")
                translation = np.array([float(nums[0]), float(nums[1]), float(nums[2])])
                continue
            if "Orientation given in set heading (quat)" in line:
                nums = line.split("Orientation given in set heading (quat)")[1].split(" ")
                orientation = Quaternion(float(nums[3]), float(nums[0]), float(nums[1]), float(nums[2]))
                continue
        self.odoFrameToGTF = InfuseTransform(translation, orientation)

    def load_gps_poses(self):

        self.gpsPoses = []
        gpsTr = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataGps.x,
                                     self.dataGps.y,
                                     self.dataGps.z,
                                     self.dataGps.qw,
                                     self.dataGps.qx,
                                     self.dataGps.qy,
                                     self.dataGps.qz):
            self.gpsPoses.append(InfuseTransform(np.array([x,y,z]), Quaternion([qw,qx,qy,qz])))
            gpsTr.append([x,y,z])
        self.gpsTr = np.array(gpsTr)

    def compute_gps_ltf(self):

        self.gpsLtfPoses = []
        gpsLtfTr = []
        for pose in self.gpsPoses:
            # pose = self.localFrameTr.inverse() * pose
            pose.translation = pose.translation - self.localFrameTr.translation # faster
            self.gpsLtfPoses.append(pose)
            gpsLtfTr.append([pose.translation[0], pose.translation[1], pose.translation[2]])
        self.gpsLtfTr = np.array(gpsLtfTr)

    def load_odometry_poses(self):

        self.odometryPoses = []
        odoTr = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataOdometry.x,
                                     self.dataOdometry.y,
                                     self.dataOdometry.z,
                                     self.dataOdometry.qw,
                                     self.dataOdometry.qx,
                                     self.dataOdometry.qy,
                                     self.dataOdometry.qz):
            self.odometryPoses.append(InfuseTransform(np.array([x,y,z]), Quaternion([qw,qx,qy,qz])))
            odoTr.append([x,y,z])
        self.odometryTr = np.array(odoTr)

    def compute_odometry_ltf(self):

        odoFrameToLTF = self.odoFrameToGTF
        odoFrameToLTF.translation = self.gpsLtfTr[0,:] - odoFrameToLTF.orientation.rotate(self.gpsInternalTr.translation)

        self.odometryLtfPoses = []
        odoLtfTr = []
        for pose in self.odometryPoses:
            pose = odoFrameToLTF * pose
            self.odometryLtfPoses.append(pose)
            odoLtfTr.append([pose.translation[0], pose.translation[1], pose.translation[2]])
        self.odometryLtfTr = np.array(odoLtfTr)

    def load_tokamak_poses(self):

        self.tokamakPoses = []
        tokamakTr = []
        for x,y,z,qw,qx,qy,qz in zip(self.dataTokamak.x,
                                     self.dataTokamak.y,
                                     self.dataTokamak.z,
                                     self.dataTokamak.qw,
                                     self.dataTokamak.qx,
                                     self.dataTokamak.qy,
                                     self.dataTokamak.qz):
            self.tokamakPoses.append(InfuseTransform(np.array([x,y,z]), Quaternion([qw,qx,qy,qz])))
            tokamakTr.append([x,y,z])
        self.tokamakTr = np.array(tokamakTr)

    def compute_robot_pose_ltf(self):
        
        interpolator = interp1d(np.array(self.dataOdometry.child_time), 
                                np.array([i for i in range(len(self.dataOdometry.child_time))]),
                                kind='nearest',
                                bounds_error=False,
                                fill_value=0,
                                assume_sorted=True)
        synchedOdoIndexes = interpolator(self.dataGps.child_time)

        self.robotLtfPoses = []
        robotLtfTr = []

        self.delayGpsOdo = []
        for gpsPose, odoIndex, gpsStamp in zip(self.gpsLtfPoses, synchedOdoIndexes, self.dataGps.child_time):
            odoPose = self.odometryLtfPoses[int(odoIndex)]
            pose = InfuseTransform(
                gpsPose.translation - odoPose.orientation.rotate(self.gpsInternalTr.translation),
                odoPose.orientation)
            self.robotLtfPoses.append(pose)
            self.delayGpsOdo.append(gpsStamp - self.dataOdometry.child_time[int(odoIndex)])
            robotLtfTr.append([pose.translation[0], pose.translation[1], pose.translation[2]])
        # for gpsPose, odoIndex in zip(self.gpsLtfPoses, synchedOdoIndexes):
        #     odoPose = self.odometryLtfPoses[int(odoIndex)]
        #     pose = InfuseTransform(
        #         gpsPose.translation - odoPose.orientation.rotate(self.gpsInternalTr.translation),
        #         odoPose.orientation)
        #     self.robotLtfPoses.append(pose)
        #     robotLtfTr.append([pose.translation[0], pose.translation[1], pose.translation[2]])
        self.robotLtfTr = np.array(robotLtfTr)

    def display(self):

        # GPS and odometry in their respective frames (not interesting...)
        fig, axes = plt.subplots(1,2, sharex=False, sharey=False)
        axes[0].plot(self.gpsTr[:,0], self.gpsTr[:,1], label="GPS UTM")
        axes[0].legend(loc="upper right")
        axes[0].set_xlabel("East (m)")
        axes[0].set_ylabel("North (m)")
        axes[0].set_aspect('equal')
        axes[0].grid()
        axes[1].plot(self.odometryTr[:,0], self.odometryTr[:,1], label="Raw odometry \"odom frame\"")
        axes[1].legend(loc="upper right")
        axes[1].set_xlabel("x (m)")
        axes[1].set_ylabel("y (m)")
        axes[1].set_aspect('equal')
        axes[1].grid()

        # GPS and odometry in LTF (more interesting)
        fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
        # axes.plot(self.gpsLtfTr[:,0], self.gpsLtfTr[:,1], '--o', label="GPS LTF")
        # axes.plot(self.odometryLtfTr[:,0], self.odometryLtfTr[:,1], '--o', label="Odometry LTF")
        axes.plot(self.tokamakTr[:,0], self.tokamakTr[:,1], '--o', label="Tokamak")
        axes.plot(self.robotLtfTr[:,0], self.robotLtfTr[:,1], '--o', label="Robot LTF")
        axes.legend(loc="upper right")
        axes.set_xlabel("East (m)")
        axes.set_ylabel("North (m)")
        axes.set_aspect('equal')
        axes.grid()

        fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
        axes.plot((np.array(self.dataGps.child_time)[0:-100] - self.dataGps.child_time[0]) / 1000000.0,
                  np.array(self.delayGpsOdo)[0:-100] / 1000.0, '--o', label="Delay Gps/Odo")
        axes.legend(loc="upper right")
        axes.set_xlabel("Gps pose index")
        axes.set_ylabel("Delay (ms)")
        axes.grid()

        plt.show(block=False)

