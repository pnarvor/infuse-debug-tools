import os
import numpy as np
import xml.etree.ElementTree as XmlTree
from pyquaternion import Quaternion
from scipy.interpolate import CubicSpline
from scipy.interpolate import Akima1DInterpolator
from scipy.interpolate import interp1d

from .Utils    import InfuseTransform
from .Metadata import Metadata

class InfusePose(InfuseTransform):

    def __init__(self, stamp=0, tr=InfuseTransform()):
        super(InfusePose, self).__init__(tr.translation,
                                         tr.orientation)
        self.stamp = stamp

    def __str__(self):
        return ("stamp       : " + str(self.stamp) + "\n"
               + super(InfusePose, self).__str__())

class DataPoseTagger:

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

    def remove_duplicate_poses(poseList, center=50000, tol=15000):

        stamps = np.array([pose.stamp for pose in poseList])
        # indexesToRemove = np.where(stamps[1:] - stamps[:-1] - center < -tol)[0]
        indexesToRemove = np.where(stamps[1:] == stamps[:-1])[0]
        print("Removing duplicates : ", len(indexesToRemove), " poses to remove") 
        for index in reversed(indexesToRemove):
            poseList.pop(index)

    def __init__(self, dataRootDir):

        self.dataRootDir = dataRootDir

        # GPS raw files
        # self.gpsDataFormatFilename       = os.path.join(self.dataRootDir, "gps/gps_pose_info_dataformat.txt")
        # self.gpsDataFilename             = os.path.join(self.dataRootDir, "gps/gps_pose_info.txt")
        # self.gpsDataFormatFilename       = os.path.join(self.dataRootDir, "velodyne/dataformat.txt")
        # self.gpsDataFilename             = os.path.join(self.dataRootDir, "velodyne/all_metadata.txt")
        self.gpsDataFormatFilename       = os.path.join(self.dataRootDir, "nav_cam/pair_dataformat.txt")
        self.gpsDataFilename             = os.path.join(self.dataRootDir, "nav_cam/pair_all_metadata.txt")

        # Odometry raw files
        self.odometryDataFormatFilename  = os.path.join(self.dataRootDir, "odometry/dataformat.txt")
        self.odometryDataFilename        = os.path.join(self.dataRootDir, "odometry/odometry.txt")

        # URDF for gps position on robot
        self.gpsUrdfFilename             = DataPoseTagger.find_urdf(os.path.join(self.dataRootDir, "../tokamak_urdf_internal_"))
        # URDF for LocalTerrainFrame ans site frame
        self.localFramesUrdfFilename     = DataPoseTagger.find_urdf(os.path.join(self.dataRootDir, "../fixed_frames_"))
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
    
    def load(self):

        self.parse_internal_urdf()
        self.parse_external_urdf()
        self.parse_heading_file()
        self.load_gps()
        self.load_odometry()
        DataPoseTagger.remove_duplicate_poses(self.gpsTr, 50000, 15000)
        DataPoseTagger.remove_duplicate_poses(self.odometryTr, 50000, 15000)

        # self.gpsPoseInterpolator = CubicSpline(np.array([pose.stamp for pose in self.gpsTr]),
        #                                        np.array([[pose.translation[0],
        #                                                   pose.translation[1],
        #                                                   pose.translation[2]]
        #                                                   for pose in self.gpsTr]))
        self.gpsPoseInterpolator = Akima1DInterpolator(
            np.array([pose.stamp for pose in self.gpsTr]),
            np.array([[pose.translation[0], pose.translation[1], pose.translation[2]] for pose in self.gpsTr]))
        # self.gpsPoseInterpolator = interp1d(np.array([pose.stamp for pose in self.gpsTr]),
        #                                     np.array([[pose.translation[0],
        #                                                pose.translation[1],
        #                                                pose.translation[2]]
        #                                                for pose in self.gpsTr]),
        #                                     axis=0)
        
    def parse_internal_urdf(self):

        root = XmlTree.parse(self.gpsUrdfFilename).getroot()
        for child in root:
            if child.attrib['name'] == "Rover2GPS":
                self.gpsInternalTr = DataPoseTagger.parse_urdf_joint(child)
        if self.gpsInternalTr is None:
            raise Exception("Error parsing", self.gpsUrdfFilename, ". File corrupted ?")

    def parse_external_urdf(self):

        root = XmlTree.parse(self.localFramesUrdfFilename).getroot()
        siteFrameTr  = None
        localFrameTr = None
        for child in root:
            if child.attrib['name'] == "SiteFrameToGlobalTerrainFrame":
                siteFrameTr  = DataPoseTagger.parse_urdf_joint(child)
            if child.attrib['name'] == "LocalTerrainFrameToSiteFrame":
                localFrameTr = DataPoseTagger.parse_urdf_joint(child)
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
        # for stamp,x,y,z,qw,qx,qy,qz in zip(self.gpsData.child_time,
        #                                    self.gpsData.x,
        #                                    self.gpsData.y,
        #                                    self.gpsData.z,
        #                                    self.gpsData.qw,
        #                                    self.gpsData.qx,
        #                                    self.gpsData.qy,
        #                                    self.gpsData.qz):
        #     self.gpsTr.append(InfusePose(stamp, InfuseTransform(np.array([x,y,z]), Quaternion(qw,qx,qy,qz))))
        for stamp,x,y,z,qw,qx,qy,qz in zip(self.gpsData.left__timestamp,
                                           self.gpsData.left__pose_fixed_robot__x,
                                           self.gpsData.left__pose_fixed_robot__y,
                                           self.gpsData.left__pose_fixed_robot__z,
                                           self.gpsData.left__pose_fixed_robot__qw,
                                           self.gpsData.left__pose_fixed_robot__qx,
                                           self.gpsData.left__pose_fixed_robot__qy,
                                           self.gpsData.left__pose_fixed_robot__qz):
            self.gpsTr.append(InfusePose(stamp, InfuseTransform(np.array([x,y,z]), Quaternion(qw,qx,qy,qz))))

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
            self.odometryTr.append(InfusePose(stamp, InfuseTransform(np.array([x,y,z]), Quaternion(qw,qx,qy,qz))))
    
    def get_gps_interpolated(self, stamps):
        
        return self.gpsPoseInterpolator(stamps)

