import os
import numpy as np
import matplotlib.pyplot as plt
import io
from pyquaternion import Quaternion
import xml.etree.ElementTree as XmlTree
from scipy.signal import medfilt
# from scipy.interpolate import NearestNDInterpolator
from scipy.interpolate import interp1d
from scipy.signal import medfilt
from scipy.signal import convolve

from .Utils    import InfuseTransform
from .Utils    import spike_detector
from .Metadata import Metadata

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

def compute_curvilinear_abscisse(poseList):
    abscisse = [0.0]
    lastPose = poseList[0]
    lastAbs = 0.0
    for pose in poseList[1:]:
        lastAbs = lastAbs + np.linalg.norm(pose.translation - lastPose.translation)
        abscisse.append(lastAbs)
        lastPose = pose
    return abscisse

def compute_speed(curvAbs, stamps):
    speed = [0.0]
    lastAbs   = curvAbs[0]
    lastStamp = stamps[0]
    for absc, stamp in zip(curvAbs[1:], stamps[1:]):
        if stamp - lastStamp <= 1e-8:
            speed.append(-1)
        else:
            speed.append(1000000.0*(absc - lastAbs) / ((stamp - lastStamp)))
        lastAbs   = absc
        lastStamp = stamp
    # return medfilt(speed, kernel_size=2*int(20.0/2)+1)
    # return medfilt(speed, kernel_size=101)
    return convolve(speed, np.ones(101) / 101.0, mode='same') 
    # return speed

class FullRobotPose:
    def __init__(self, stamp, tr, gpsStddev, curveAbs, speed):
        self.stamp = stamp
        self.tr = tr
        self.gpsStddev = gpsStddev
        self.curveAbs  = curveAbs
        self.speed     = speed 

class RobotPoseData:

    def __init__(self, dataRootDir):

        # Example of dataRootDir      : "/media/data/log_data_acquisition/raw_data"
        self.dataRootDir = dataRootDir

        # raw file path
        self.gpsDataFormatFilename      = os.path.join(self.dataRootDir, "gps/gps_pose_info_dataformat.txt")
        self.gpsDataFilename            = os.path.join(self.dataRootDir, "gps/gps_pose_info.txt")
        self.robotLtfPoses = []
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
        self.gpsPoses             = []
        self.gpsLtfPoses          = []
        self.odometryPoses        = []
        self.odometryLtfPoses     = []
        self.tokamakPoses         = []
        self.robotLtfPoses        = []
        self.minTime              = -1
        self.poseInterpolator     = None 
        self.robotLtfPoses        = []
        self.odometryInterpolator = None
        self.robotPoseCurveAbs    = []
        self.odometryCurveAbs     = []
        self.robotLtfSpeed        = []
        self.odometrySpeed        = []

        # data for display
        self.gpsTr         = np.empty([0])
        self.gpsLtfTr      = np.empty([0])
        self.odometryTr    = np.empty([0])
        self.odometryLtfTr = np.empty([0])
        self.tokamakTr     = np.empty([0])

    def load(self):

        print("Loading data files... ", end="", flush=True)
        self.load_files()
        print("Done !")

        print("Computing display variables... ", end="", flush=True)
        self.load_gps_poses()
        self.load_odometry_poses()
        self.load_tokamak_poses()
        self.compute_gps_ltf()
        self.compute_odometry_ltf()
        self.compute_robot_pose_ltf()
        self.robotPoseCurveAbs = compute_curvilinear_abscisse(self.robotLtfPoses)
        self.odometryCurveAbs  = compute_curvilinear_abscisse(self.odometryPoses)
        self.robotLtfSpeed     = compute_speed(self.robotPoseCurveAbs, self.dataGps.child_time)
        self.odometrySpeed     = compute_speed(self.odometryCurveAbs , self.dataOdometry.child_time)
        print("Done !")

    def load_files(self):
        
        self.parse_internal_urdf()
        self.parse_external_urdf()
        self.parse_heading_file()
        self.dataGps.parse_metadata(self.gpsDataFormatFilename, self.gpsDataFilename)
        self.dataOdometry.parse_metadata(self.odometryDataFormatFilename, self.odometryDataFilename)
        self.dataTokamak.parse_metadata(self.tokamakDataFormatFilename, self.tokamakDataFilename)
        self.minTime = self.dataTokamak.child_time[0]

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

    def build_pose_interpolator(self):

        stamps = np.array(self.dataGps.child_time)
        indexesToRemove = np.where(stamps[1:] == stamps[:-1])[0]
        gpsUniqueStampIndexes = [i for i in range(len(self.dataGps.child_time))]
        gpsUniqueStamps = [t for t in self.dataGps.child_time]
        print("Removing duplicates : ", len(indexesToRemove), " poses to remove") 
        for index in reversed(indexesToRemove):
            gpsUniqueStampIndexes.pop(index)
            gpsUniqueStamps.pop(index)

        self.poseInterpolator = interp1d(gpsUniqueStamps,
                                         gpsUniqueStampIndexes,
                                         kind='nearest',
                                         bounds_error=False,
                                         fill_value=0,
                                         assume_sorted=True)

    def interpolate(self, stamps):

        if self.poseInterpolator is None:
            self.build_pose_interpolator()

        poseIndexes = self.poseInterpolator(stamps)
        return [FullRobotPose(self.dataGps.child_time[int(i)],
                              self.robotLtfPoses[int(i)],
                              np.array([self.dataGps.easting_sigma[int(i)],
                                        self.dataGps.northing_sigma[int(i)],
                                        self.dataGps.height_sigma[int(i)]]),
                                        self.robotPoseCurveAbs[int(i)],
                                        self.robotLtfSpeed[int(i)]) for i in poseIndexes]

    def build_odometry_interpolator(self):

        self.odometryInterpolator = interp1d(self.dataOdometry.child_time,
                                             [i for i in range(len(self.dataOdometry.child_time))],
                                             kind='nearest',
                                             bounds_error=False,
                                             fill_value=0,
                                             assume_sorted=True)

    def interpolate_odometry(self, stamps):

        if self.odometryInterpolator is None:
            self.build_odometry_interpolator()

        poseIndexes = self.odometryInterpolator(stamps)
        return [FullRobotPose(self.dataOdometry.child_time[int(i)],
                              self.odometryPoses[int(i)],
                              np.array([0.0, 0.0, 0.0]),
                              self.odometryCurveAbs[int(i)],
                              self.odometrySpeed[int(i)]) for i in poseIndexes]

    def display(self, verbose=False, blocking=False):

        if verbose:
            # GPS and odometry in their respective frames (not interesting...)
            fig, axes = plt.subplots(1,2, sharex=False, sharey=False)
            axes[0].set_title("RobotPoseData : GPS and Odometry")
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

            fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
            axes.set_title("RobotPoseData : Delay between GPS Position and Odometry Attitude")
            axes.plot((np.array(self.dataGps.child_time)[0:-100] - self.dataGps.child_time[0]) / 1000000.0,
                      np.array(self.delayGpsOdo)[0:-100] / 1000.0, '--o', label="Delay Gps/Odo", markeredgewidth=0.0)
            axes.legend(loc="upper right")
            axes.set_xlabel("Gps pose index")
            axes.set_ylabel("Delay (ms)")
            axes.grid()

            fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
            axes.set_title("RobotPoseData : Curvilinear abscisses")
            axes.plot((np.array(self.dataOdometry.child_time) - self.minTime) / 1000000.0, self.odometryCurveAbs, '--o', label="Odometry curv abs", markeredgewidth=0.0)
            axes.plot((np.array(self.dataGps.child_time) - self.minTime) / 1000000.0, self.robotPoseCurveAbs, '--o', label="GPS curv abs", markeredgewidth=0.0)
            axes.legend(loc="upper right")
            axes.set_xlabel("Mission time (s)")
            axes.set_ylabel("Curv abscisse (m)")
            axes.grid()

            fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
            axes[0].set_title("RobotPoseData : Original Tokamak and a posteriori robot pose")
            axes[0].plot((np.array(self.dataTokamak.child_time) - self.minTime) / 1000000.0, self.tokamakTr[:,0], '--o', label="Tokamak", markeredgewidth=0.0)
            axes[0].plot((np.array(self.dataGps.child_time) - self.minTime) / 1000000.0, self.robotLtfTr[:,0], '--o', label="Recomp Tokamak", markeredgewidth=0.0)
            axes[0].legend(loc="upper right")
            # axes[0].set_xlabel("Mission time (s)")
            axes[0].set_ylabel("East (m)")
            axes[0].grid()
            axes[1].plot((np.array(self.dataTokamak.child_time) - self.minTime) / 1000000.0, self.tokamakTr[:,1], '--o', label="Tokamak", markeredgewidth=0.0)
            axes[1].plot((np.array(self.dataGps.child_time) - self.minTime) / 1000000.0, self.robotLtfTr[:,1], '--o', label="Recomp tokamak", markeredgewidth=0.0)
            axes[1].legend(loc="upper right")
            # axes[1].set_xlabel("Mission time (s)")
            axes[1].set_ylabel("North (m)")
            axes[1].grid()
            axes[2].plot((np.array(self.dataTokamak.child_time) - self.minTime) / 1000000.0, self.tokamakTr[:,2], '--o', label="Tokamak", markeredgewidth=0.0)
            axes[2].plot((np.array(self.dataGps.child_time) - self.minTime) / 1000000.0, self.robotLtfTr[:,2], '--o', label="Recomp tokamak", markeredgewidth=0.0)
            axes[2].legend(loc="upper right")
            axes[2].set_xlabel("Mission time (s)")
            axes[2].set_ylabel("Elevation (m)")
            axes[2].grid()

        # GPS and odometry in LTF (more interesting)
        fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
        axes.set_title("RobotPoseData : All traces (transformed to LocalTerrainFrame")
        axes.plot(self.gpsLtfTr[:,0], self.gpsLtfTr[:,1], '--o', label="GPS LTF", markeredgewidth=0.0)
        axes.plot(self.odometryLtfTr[:,0], self.odometryLtfTr[:,1], '--o', label="Odometry LTF", markeredgewidth=0.0)
        axes.plot(self.tokamakTr[:,0], self.tokamakTr[:,1], '--o', label="Tokamak")
        axes.plot(self.robotLtfTr[:,0], self.robotLtfTr[:,1], '--o', label="Robot pose recomposed", markeredgewidth=0.0)
        axes.legend(loc="upper right")
        axes.set_xlabel("East (m)")
        axes.set_ylabel("North (m)")
        axes.set_aspect('equal')
        axes.grid()


        plt.show(block=blocking)

