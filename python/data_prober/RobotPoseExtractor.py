import os
import numpy as np
import xml.etree.ElementTree as XmlTree
from pyquaternion import Quaternion

from .Metadata import Metadata
from .InfuseTransform import InfuseTransform

class UsableIterator:

    def __init__(self, listToIterate):
        self.value    = listToIterate[0]
        self.iterator = iter(listToIterate)

    def __iter__(self):
        return self

    def __next__(self):
        self.value = next(self.iterator)

class RobotPoseExtractor:

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

    def __init__(self, dataRootDir):

        self.dataRootDir = dataRootDir

        # GPS raw files
        self.gpsDataFormatFilename       = os.path.join(self.dataRootDir, "gps/gps_pose_info_dataformat.txt")
        self.gpsDataFilename             = os.path.join(self.dataRootDir, "gps/gps_pose_info.txt")

        # Odometry raw files
        self.odometryDataFormatFilename  = os.path.join(self.dataRootDir, "odometry/dataformat.txt")
        self.odometryDataFilename        = os.path.join(self.dataRootDir, "odometry/odometry.txt")

        # URDF for gps position on robot
        self.gpsUrdfFilename             = RobotPoseExtractor.find_urdf(os.path.join(self.dataRootDir, "../tokamak_urdf_internal_"))
        # URDF for LocalTerrainFrame ans site frame
        self.localFramesUrdfFilename     = RobotPoseExtractor.find_urdf(os.path.join(self.dataRootDir, "../fixed_frames_"))
        # Output file of set_heading process
        self.headingFilename             = os.path.join(self.dataRootDir, "../test_parking.txt")

        # Fixed frames transforms
        self.localFrameTr  = None # LocalTerrainFrame (LTF) to GlobalTerrainFrame (GTF)
        self.gpsInternalTr = None # GPS antenna position in robot (relative to RobotBodyFrame, RBF)
        self.odoFrameToGTF = None # OdometryFrame (frame in which RMP export its data) to GTF

        self.gpsData         = Metadata()
        self.gpsTr           = []

        self.odometryData    = Metadata()
        self.odometryTr      = []

        self.gpsSynchronized = [] # Contains GPS poses in GTF synchronised (to closest in time) to odometry poses
        self.odoSynchronized = [] # Contains GPS poses in GTF synchronised (to closest in time) to odometry poses
        self.robotPoses      = [] # Final full RBF poses in LTF
    
    def load(self):

        self.parse_internal_urdf()
        self.parse_external_urdf()
        self.parse_heading_file()
        self.load_gps()
        self.load_odometry()
        self.synchronize_gps_on_odometry()
        self.synchronize_odometry_on_gps()
        self.compute_robot_pose()
        self.interpolate()
        
    def parse_internal_urdf(self):

        root = XmlTree.parse(self.gpsUrdfFilename).getroot()
        for child in root:
            if child.attrib['name'] == "Rover2GPS":
                self.gpsInternalTr = RobotPoseExtractor.parse_urdf_joint(child)
        if self.gpsInternalTr is None:
            raise Exception("Error parsing", self.gpsUrdfFilename, ". File corrupted ?")

    def parse_external_urdf(self):

        root = XmlTree.parse(self.localFramesUrdfFilename).getroot()
        siteFrameTr  = None
        localFrameTr = None
        for child in root:
            if child.attrib['name'] == "SiteFrameToGlobalTerrainFrame":
                siteFrameTr  = RobotPoseExtractor.parse_urdf_joint(child)
            if child.attrib['name'] == "LocalTerrainFrameToSiteFrame":
                localFrameTr = RobotPoseExtractor.parse_urdf_joint(child)
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

    def load_gps(self):

        self.gpsData.parse_metadata(self.gpsDataFormatFilename, self.gpsDataFilename)
        for stamp,x,y,z,qw,qx,qy,qz in zip(self.gpsData.child_time,
                                           self.gpsData.x,
                                           self.gpsData.y,
                                           self.gpsData.z,
                                           self.gpsData.qw,
                                           self.gpsData.qx,
                                           self.gpsData.qy,
                                           self.gpsData.qz):
            self.gpsTr.append((stamp, InfuseTransform(np.array([x,y,z]), Quaternion(qw,qx,qy,qz)),))

    def load_odometry(self):

        self.odometryData.parse_metadata(self.odometryDataFormatFilename, self.odometryDataFilename)
        for stamp,x,y,z,qw,qx,qy,qz in zip(self.odometryData.child_time,
                                           self.odometryData.x,
                                           self.odometryData.y,
                                           self.odometryData.z,
                                           self.odometryData.qw,
                                           self.odometryData.qx,
                                           self.odometryData.qy,
                                           self.odometryData.qz):
            self.odometryTr.append((stamp, InfuseTransform(np.array([x,y,z]), Quaternion(qw,qx,qy,qz)),))
    
    def synchronize_gps_on_odometry(self):

        gpsStamps = np.array(self.gpsData.child_time)
        for odoPose in self.odometryTr:
            self.gpsSynchronized.append(
                self.gpsTr[np.abs(gpsStamps - odoPose[0]).argmin()])

    def synchronize_odometry_on_gps(self):

        odoStamps = np.array(self.odometryData.child_time)
        for gpsPose in self.gpsTr:
            self.odoSynchronized.append(
                self.odometryTr[np.abs(odoStamps - gpsPose[0]).argmin()])

    def compute_robot_pose(self):

        # for odo, gps in zip(self.odometryTr, self.gpsSynchronized):
        for odo, gps in zip(self.odoSynchronized, self.gpsTr):

            odoTr = odo[1]
            gpsTr = gps[1]

            translation = gpsTr.translation - self.localFrameTr.translation
            orientation = self.odoFrameToGTF.orientation * odoTr.orientation
            pose = InfuseTransform(translation
                                   - orientation.rotate(self.gpsInternalTr.translation),
                                   orientation)

            self.robotPoses.append((odo[0], gps[0], pose,))

    # def interpolate(self, frequency):

        




