import os
import numpy as np
import io
from pyquaternion import Quaternion
from scipy.signal import medfilt

from .Metadata import Metadata
from .InfuseTransform import InfuseTransform

def get_pcd_header(filename):

    res = ""
    pcdFile = io.open(filename, "rb")
    fileBuffer = io.BufferedReader(pcdFile, buffer_size=1)
    # textReader = io.TextIOWrapper(fileBuffer, encoding="ascii")
    textReader = io.TextIOWrapper(pcdFile, encoding="ascii", errors='backslashreplace')
   
    for i in range(400):
        try:
            res += textReader.read(1)
        except Exception as e:
            print(e) 
            break
    
    return res
 
def spike_detector(data):
   return np.abs(data - medfilt(data, kernel_size=3)) 

def parse_export_plan(filename):

    f = open(filename, 'r')
    exportPlan = yaml.safe_load(f)
    if not "intervals_to_export" in exportPlan.keys():
        raise Exception("No intervals to export in export plan file")
    if not isinstance(exportPlan['intervals_to_export'], list):
        raise Exception("Export plan parsing error : "
                        + "intervals_to_export item must be a [] list.")
    if not isinstance(exportPlan['intervals_to_export'][0], list):
        intervalsToExport = [exportPlan['intervals_to_export']]
    else:
        intervalsToExport = exportPlan['intervals_to_export']
    if not "data_to_remove" in exportPlan.keys():
        print("Warning : No data to remove in export plan"
              + "Have you got a perfect dataset ?!")
        return
    if not isinstance(exportPlan['data_to_remove'], list):
        dataToRemove = [exportPlan['data_to_remove']]
    else:
        dataToRemove = exportPlan['data_to_remove']

    return intervalsToExport, dataToRemove;

class DataCleaner:
    
    def spike_detector_filter(data):
       return np.abs(data - medfilt(data, kernel_size=3)) 

    def __init__(self, dataRootDir):

        self.dataRootDir = dataRootDir

        # Front cam raw files
        self.frontLeftDataFormatFilename           = os.path.join(self.dataRootDir, "front_cam/left/left_dataformat.txt")
        self.frontLeftDataFilename                 = os.path.join(self.dataRootDir, "front_cam/left/left_all_metadata.txt")
        self.frontLeftIntegrityDataFormatFilename  = os.path.join(self.dataRootDir, "front_cam/left/image_integrity_dataformat.txt")
        self.frontLeftIntegrityDataFilename        = os.path.join(self.dataRootDir, "front_cam/left/image_integrity.txt")
        self.frontRightDataFormatFilename          = os.path.join(self.dataRootDir, "front_cam/right/right_dataformat.txt")
        self.frontRightDataFilename                = os.path.join(self.dataRootDir, "front_cam/right/right_all_metadata.txt")
        self.frontRightIntegrityDataFormatFilename = os.path.join(self.dataRootDir, "front_cam/right/image_integrity_dataformat.txt")
        self.frontRightIntegrityDataFilename       = os.path.join(self.dataRootDir, "front_cam/right/image_integrity.txt")
        self.frontDisparityDataFormatFilename      = os.path.join(self.dataRootDir, "front_disparity/disparity_dataformat.txt")
        self.frontDisparityDataFilename            = os.path.join(self.dataRootDir, "front_disparity/disparity_all_metadata.txt")

        # Rear cam raw files
        self.rearLeftDataFormatFilename           = os.path.join(self.dataRootDir, "rear_cam/left/left_dataformat.txt")
        self.rearLeftDataFilename                 = os.path.join(self.dataRootDir, "rear_cam/left/left_all_metadata.txt")
        self.rearLeftIntegrityDataFormatFilename  = os.path.join(self.dataRootDir, "rear_cam/left/image_integrity_dataformat.txt")
        self.rearLeftIntegrityDataFilename        = os.path.join(self.dataRootDir, "rear_cam/left/image_integrity.txt")
        self.rearRightDataFormatFilename          = os.path.join(self.dataRootDir, "rear_cam/right/right_dataformat.txt")
        self.rearRightDataFilename                = os.path.join(self.dataRootDir, "rear_cam/right/right_all_metadata.txt")
        self.rearRightIntegrityDataFormatFilename = os.path.join(self.dataRootDir, "rear_cam/right/image_integrity_dataformat.txt")
        self.rearRightIntegrityDataFilename       = os.path.join(self.dataRootDir, "rear_cam/right/image_integrity.txt")
        self.rearDisparityDataFormatFilename      = os.path.join(self.dataRootDir, "rear_disparity/disparity_dataformat.txt")
        self.rearDisparityDataFilename            = os.path.join(self.dataRootDir, "rear_disparity/disparity_all_metadata.txt")

        # Nav cam raw files
        self.navLeftDataFormatFilename           = os.path.join(self.dataRootDir, "nav_cam/left/left_dataformat.txt")
        self.navLeftDataFilename                 = os.path.join(self.dataRootDir, "nav_cam/left/left_all_metadata.txt")
        self.navLeftIntegrityDataFormatFilename  = os.path.join(self.dataRootDir, "nav_cam/left/image_integrity_dataformat.txt")
        self.navLeftIntegrityDataFilename        = os.path.join(self.dataRootDir, "nav_cam/left/image_integrity.txt")
        self.navRightDataFormatFilename          = os.path.join(self.dataRootDir, "nav_cam/right/right_dataformat.txt")
        self.navRightDataFilename                = os.path.join(self.dataRootDir, "nav_cam/right/right_all_metadata.txt")
        self.navRightIntegrityDataFormatFilename = os.path.join(self.dataRootDir, "nav_cam/right/image_integrity_dataformat.txt")
        self.navRightIntegrityDataFilename       = os.path.join(self.dataRootDir, "nav_cam/right/image_integrity.txt")
        self.navDisparityDataFormatFilename      = os.path.join(self.dataRootDir, "nav_disparity/disparity_dataformat.txt")
        self.navDisparityDataFilename            = os.path.join(self.dataRootDir, "nav_disparity/disparity_all_metadata.txt")

        #GPS raw files
        self.gpsDataFormatFilename       = os.path.join(self.dataRootDir, "gps/gps_pose_info_dataformat.txt")
        self.gpsDataFilename             = os.path.join(self.dataRootDir, "gps/gps_pose_info.txt")
        #Odometry raw files
        self.odometryDataFormatFilename  = os.path.join(self.dataRootDir, "odometry/dataformat.txt")
        self.odometryDataFilename        = os.path.join(self.dataRootDir, "odometry/odometry.txt")
        #Delta odometry raw files
        self.deltaOdometryDataFormatFilename  = os.path.join(self.dataRootDir, "odom_delta/dataformat.txt")
        self.deltaOdometryDataFilename        = os.path.join(self.dataRootDir, "odom_delta/odom_delta.txt")
        #Tokamak raw files
        self.tokamakDataFormatFilename   = os.path.join(self.dataRootDir, "tokamak/dataformat.txt")
        self.tokamakDataFilename         = os.path.join(self.dataRootDir, "tokamak/tokamak.txt")
        self.headingFilename             = os.path.join(self.dataRootDir, "../test_parking.txt")
        #Velodyne raw files
        self.velodyneDataFormatFilename  = os.path.join(self.dataRootDir, "velodyne/dataformat.txt")
        self.velodyneDataFilename        = os.path.join(self.dataRootDir, "velodyne/all_metadata.txt")
        self.velodynePCDPath             = os.path.join(self.dataRootDir, "velodyne/data/")

        self.hasFrontData            = False
        self.dataFrontLeft           = Metadata()
        self.dataFrontRight          = Metadata()
        self.dataFrontDisparity      = Metadata()
        self.dataFrontLeftIntegrity  = Metadata()
        self.dataFrontRightIntegrity = Metadata()
        self.frontStereoDesync       = np.empty([0])
        self.frontStereoStamps       = np.empty([0])
        self.frontMinTime            = -1
        self.frontDisparityScore     = np.empty([0])
        self.frontLeftIntegrity      = np.empty([0])
        self.frontRightIntegrity     = np.empty([0])
        self.frontTotalScore         = np.empty([0])
        self.frontRightToLeft        = InfuseTransform()

        self.hasRearData            = False
        self.dataRearLeft           = Metadata()
        self.dataRearRight          = Metadata()
        self.dataRearDisparity      = Metadata()
        self.dataRearLeftIntegrity  = Metadata()
        self.dataRearRightIntegrity = Metadata()
        self.rearStereoDesync       = np.empty([0])
        self.rearStereoStamps       = np.empty([0])
        self.rearMinTime            = -1
        self.rearDisparityScore     = np.empty([0])
        self.rearLeftIntegrity      = np.empty([0])
        self.rearRightIntegrity     = np.empty([0])
        self.rearTotalScore         = np.empty([0])
        self.rearRightToLeft        = InfuseTransform()

        self.hasNavData            = False
        self.dataNavLeft           = Metadata()
        self.dataNavRight          = Metadata()
        self.dataNavDisparity      = Metadata()
        self.dataNavLeftIntegrity  = Metadata()
        self.dataNavRightIntegrity = Metadata()
        self.navStereoDesync       = np.empty([0])
        self.navStereoStamps       = np.empty([0])
        self.navMinTime            = -1
        self.navDisparityScore     = np.empty([0])
        self.navLeftIntegrity      = np.empty([0])
        self.navRightIntegrity     = np.empty([0])
        self.navTotalScore         = np.empty([0])
        self.navRightToLeft        = InfuseTransform()
        self.navPositionDiff       = np.empty([0])

        # Velodyne
        self.hasVelodyneData   = False
        self.dataVelodyne      = Metadata()
        self.velodyneMinTime   = -1
        self.velodyneCloudTime = np.empty([0])
        self.velodynePeriod    = np.empty([0])
        self.velodyneDesync    = np.empty([0])
        self.velodyneNbPoints  = np.empty([0])

        # GPS
        self.dataGPS   = Metadata()
        self.gpsTime   = np.empty([0])
        self.gpsX      = np.empty([0])
        self.gpsY      = np.empty([0])
        self.gpsZ      = np.empty([0])
        self.gpsPeriod = np.empty([0])
        self.gpsSpeed  = np.empty([0])
        
        # Odometry
        self.dataOdometry   = Metadata()
        self.odometryTime   = np.empty([0])
        self.odometryX      = np.empty([0])
        self.odometryY      = np.empty([0])
        self.odometryZ      = np.empty([0])
        self.odometryPeriod = np.empty([0])
        self.odometrySpeed  = np.empty([0])

        # Delta odometry
        self.dataDeltaOdometry        = Metadata()
        self.deltaOdometryTime        = np.empty([0])
        self.deltaOdometryX           = np.empty([0])
        self.deltaOdometryY           = np.empty([0])
        self.deltaOdometryZ           = np.empty([0])
        self.deltaOdometryIntegratedX = np.empty([0])
        self.deltaOdometryIntegratedY = np.empty([0])
        self.deltaOdometryIntegratedZ = np.empty([0])
        self.deltaOdometryPeriod      = np.empty([0])
        self.deltaOdometrySpeed       = np.empty([0])

        # Tokamak
        self.dataTokamak   = Metadata()
        self.tokamakTime   = np.empty([0])
        self.tokamakX      = np.empty([0])
        self.tokamakY      = np.empty([0])
        self.tokamakZ      = np.empty([0])
        self.tokamakPeriod = np.empty([0])
        self.tokamakSpeed  = np.empty([0])

        # Others
        self.odoFrameToGTF  = InfuseTransform()

    # Stereo desync #################################### 
    def compute_front_stereo_desync(self):
        
        try:
            self.dataFrontLeft.parse_metadata(self.frontLeftDataFormatFilename,   self.frontLeftDataFilename)
            self.dataFrontRight.parse_metadata(self.frontRightDataFormatFilename,   self.frontRightDataFilename)
        except:
            print("No front_cam metadata")
            return
        
        self.frontStereoDesync = np.abs(self.dataFrontRight.get_nparray('timestamp') - self.dataFrontLeft.get_nparray('timestamp'))
        self.frontStereoStamps = self.dataFrontLeft.get_nparray('timestamp')
        self.frontMinTime = self.frontStereoStamps[0]
        self.hasFrontData = True
        
    def compute_rear_stereo_desync(self):
        
        try:
            self.dataRearLeft.parse_metadata(self.rearLeftDataFormatFilename,   self.rearLeftDataFilename)
            self.dataRearRight.parse_metadata(self.rearRightDataFormatFilename,   self.rearRightDataFilename)
        except:
            print("No rear_cam metadata")
            return

        self.rearStereoDesync = np.abs(self.dataRearRight.get_nparray('timestamp') - self.dataRearLeft.get_nparray('timestamp'))
        self.rearStereoStamps = self.dataRearLeft.get_nparray('timestamp')
        self.rearMinTime = self.rearStereoStamps[0]
        self.hasRearData = True

    def compute_nav_stereo_desync(self):

        try:
            self.dataNavLeft.parse_metadata(self.navLeftDataFormatFilename,   self.navLeftDataFilename)
            self.dataNavRight.parse_metadata(self.navRightDataFormatFilename,   self.navRightDataFilename)
        except:
            print("No nav_cam metadata")
            return

        self.navStereoDesync = np.abs(self.dataNavRight.get_nparray('timestamp') - self.dataNavLeft.get_nparray('timestamp'))
        self.navStereoStamps = self.dataNavLeft.get_nparray('timestamp')
        self.navMinTime = self.navStereoStamps[0]
        self.hasNavData = True
       
    # Stereo desync #################################### 
    def compute_front_disparity_score(self):
        
        try:
            self.dataFrontDisparity.parse_metadata(self.frontDisparityDataFormatFilename, self.frontDisparityDataFilename)
        except:
            print("No front_cam disparity metadata")
            return

        self.frontDisparityScore = 100 - self.dataFrontDisparity.get_nparray('percentage_of_paired_pixels')
        self.frontDisparityScore = self.frontDisparityScore - np.amin(self.frontDisparityScore)

    def compute_rear_disparity_score(self):

        try:
            self.dataRearDisparity.parse_metadata(self.rearDisparityDataFormatFilename, self.rearDisparityDataFilename)
        except:
            print("No rear_cam disparity metadata")
            return

        self.rearDisparityScore = 100 - self.dataRearDisparity.get_nparray('percentage_of_paired_pixels')
        self.rearDisparityScore = self.rearDisparityScore - np.amin(self.rearDisparityScore)

    def compute_nav_disparity_score(self):

        try:
            self.dataNavDisparity.parse_metadata(self.navDisparityDataFormatFilename, self.navDisparityDataFilename)
        except:
            print("No front_cam disparity metadata")
            return

        self.navDisparityScore = 100 - self.dataNavDisparity.get_nparray('percentage_of_paired_pixels')
        self.navDisparityScore = self.navDisparityScore - np.amin(self.navDisparityScore)

    # Image integrity
    def compute_front_integrity(self):

        try:
            self.dataFrontLeftIntegrity.parse_metadata(self.frontLeftIntegrityDataFormatFilename, self.frontLeftIntegrityDataFilename)
            self.dataFrontRightIntegrity.parse_metadata(self.frontRightIntegrityDataFormatFilename, self.frontRightIntegrityDataFilename)
        except:
            print("No front_cam integrity metadata")
            return

        self.frontLeftIntegrity = self.dataFrontLeftIntegrity.get_nparray('score')
        self.frontRightIntegrity = self.dataFrontRightIntegrity.get_nparray('score')

    def compute_rear_integrity(self):
        
        try:
            self.dataRearLeftIntegrity.parse_metadata(self.rearLeftIntegrityDataFormatFilename, self.rearLeftIntegrityDataFilename)
            self.dataRearRightIntegrity.parse_metadata(self.rearRightIntegrityDataFormatFilename, self.rearRightIntegrityDataFilename)
        except:
            print("No rear_cam integrity metadata")
            return

        self.rearLeftIntegrity = self.dataRearLeftIntegrity.get_nparray('score')
        self.rearRightIntegrity = self.dataRearRightIntegrity.get_nparray('score')

    def compute_nav_integrity(self):
        
        try:
            self.dataNavLeftIntegrity.parse_metadata(self.navLeftIntegrityDataFormatFilename, self.navLeftIntegrityDataFilename)
            self.dataNavRightIntegrity.parse_metadata(self.navRightIntegrityDataFormatFilename, self.navRightIntegrityDataFilename)
        except:
            print("No rear_cam integrity metadata")
            return

        self.navLeftIntegrity = self.dataNavLeftIntegrity.get_nparray('score')
        self.navLeftIntegrity = self.navLeftIntegrity - np.amin(self.dataNavLeftIntegrity.get_nparray('score'))
        self.navRightIntegrity = self.dataNavRightIntegrity.get_nparray('score')
        self.navRightIntegrity = self.navRightIntegrity - np.amin(self.dataNavRightIntegrity.get_nparray('score'))

    # Stereo pair total score
    # DEPRECATED !
    def compute_front_total_score(self):

        dataArray = np.vstack((self.frontStereoDesync, self.frontDisparityScore, self.frontLeftIntegrity, self.frontRightIntegrity))

        sig = np.zeros([4,4])
        sig[0,0] = 1.0 / np.dot(dataArray[0,:], dataArray[0,:])
        sig[1,1] = 1.0 / np.dot(dataArray[1,:], dataArray[1,:])
        sig[2,2] = 1.0 / np.dot(dataArray[2,:], dataArray[2,:])
        sig[3,3] = 1.0 / np.dot(dataArray[3,:], dataArray[3,:])

        self.frontTotalScore = 1 - np.exp(-np.sum(np.multiply(dataArray, sig @ dataArray), axis=0))

    # DEPRECATED !
    def compute_rear_total_score(self):

        dataArray = np.vstack((self.rearStereoDesync, self.rearDisparityScore, self.rearLeftIntegrity, self.rearRightIntegrity))

        sig = np.zeros([4,4])
        sig[0,0] = 1.0 / np.dot(dataArray[0,:], dataArray[0,:])
        sig[1,1] = 1.0 / np.dot(dataArray[1,:], dataArray[1,:])
        sig[2,2] = 1.0 / np.dot(dataArray[2,:], dataArray[2,:])
        sig[3,3] = 1.0 / np.dot(dataArray[3,:], dataArray[3,:])

        self.rearTotalScore = 1 - np.exp(-np.sum(np.multiply(dataArray, sig @ dataArray), axis=0))

    # DEPRECATED !
    def compute_nav_total_score(self):

        dataArray = np.vstack((self.navStereoDesync, self.navDisparityScore, self.navLeftIntegrity, self.navRightIntegrity))

        sig = np.zeros([4,4])
        sig[0,0] = 1.0 / np.dot(dataArray[0,:], dataArray[0,:])
        sig[1,1] = 1.0 / np.dot(dataArray[1,:], dataArray[1,:])
        sig[2,2] = 1.0 / np.dot(dataArray[2,:], dataArray[2,:])
        sig[3,3] = 1.0 / np.dot(dataArray[3,:], dataArray[3,:])

        self.navTotalScore = 1 - np.exp(-np.sum(np.multiply(dataArray, sig @ dataArray), axis=0))

    def compute_nav_position_diff(self):

        x = self.dataNavLeft.get_nparray('pose_robot_sensor__x');
        y = self.dataNavLeft.get_nparray('pose_robot_sensor__y');
        self.navPositionDiff = (DataCleaner.spike_detector_filter(x)
                                + DataCleaner.spike_detector_filter(y))

    def compute_gps(self):

        try:
            self.dataGPS.parse_metadata(self.gpsDataFormatFilename,   self.gpsDataFilename)
        except:
            print("No gps data")
            return

        self.gpsTime = self.dataGPS.get_nparray('child_time')
        self.gpsX    = self.dataGPS.get_nparray('x')
        self.gpsY    = self.dataGPS.get_nparray('y')
        self.gpsZ    = self.dataGPS.get_nparray('z')

        self.gpsPeriod = self.gpsTime[1:] - self.gpsTime[:-1]
        dx = self.gpsX[1:] - self.gpsX[:-1]
        dy = self.gpsY[1:] - self.gpsY[:-1]
        dz = self.gpsZ[1:] - self.gpsZ[:-1]

        self.gpsSpeed = np.nan_to_num(np.sqrt(dx*dx + dy*dy + dz*dz) / self.gpsPeriod)

    def compute_odometry(self):

        try:
            self.dataOdometry.parse_metadata(self.odometryDataFormatFilename, self.odometryDataFilename)
        except:
            print("No odometry data")
            return

        self.odometryTime = self.dataOdometry.get_nparray('child_time')
        self.odometryX    = self.dataOdometry.get_nparray('x')
        self.odometryY    = self.dataOdometry.get_nparray('y')
        self.odometryZ    = self.dataOdometry.get_nparray('z')

        self.odometryPeriod = self.odometryTime[1:] - self.odometryTime[:-1]
        dx = self.odometryX[1:] - self.odometryX[:-1]
        dy = self.odometryY[1:] - self.odometryY[:-1]
        dz = self.odometryZ[1:] - self.odometryZ[:-1]

        self.odometrySpeed = np.nan_to_num(np.sqrt(dx*dx + dy*dy + dz*dz) / self.odometryPeriod)

    def compute_delta_odometry(self):

        try:
            self.dataDeltaOdometry.parse_metadata(self.deltaOdometryDataFormatFilename,   self.deltaOdometryDataFilename)
        except:
            print("No odometry data")
            return

        self.deltaOdometryTime = self.dataDeltaOdometry.get_nparray('child_time')
        self.deltaOdometryX    = self.dataDeltaOdometry.get_nparray('x')
        self.deltaOdometryY    = self.dataDeltaOdometry.get_nparray('y')
        self.deltaOdometryZ    = self.dataDeltaOdometry.get_nparray('z')

        self.deltaOdometryPeriod = self.deltaOdometryTime[1:] - self.deltaOdometryTime[:-1]
        dx = self.deltaOdometryX[1:] - self.deltaOdometryX[:-1]
        dy = self.deltaOdometryY[1:] - self.deltaOdometryY[:-1]
        dz = self.deltaOdometryZ[1:] - self.deltaOdometryZ[:-1]

        self.deltaOdometrySpeed = np.nan_to_num(np.sqrt(dx*dx + dy*dy + dz*dz) / self.deltaOdometryPeriod)

        x  = np.array([0,0,0])
        self.deltaOdometryIntegratedX = np.empty([len(self.deltaOdometryX) + 1])
        self.deltaOdometryIntegratedY = np.empty([len(self.deltaOdometryX) + 1])
        self.deltaOdometryIntegratedZ = np.empty([len(self.deltaOdometryX) + 1])
        self.deltaOdometryIntegratedX[0] = x[0]
        self.deltaOdometryIntegratedY[0] = x[1]
        self.deltaOdometryIntegratedZ[0] = x[2]

        qw = np.array(self.dataDeltaOdometry.qw)
        qx = np.array(self.dataDeltaOdometry.qx)
        qy = np.array(self.dataDeltaOdometry.qy)
        qz = np.array(self.dataDeltaOdometry.qz)
        q0 = Quaternion()

        # Integrating to check (validity of the data)
        for i in range(len(self.deltaOdometryX)):

            x[0] = self.deltaOdometryX[i]
            x[1] = self.deltaOdometryY[i]
            x[2] = self.deltaOdometryZ[i]
            x = q0.rotate(x)

            self.deltaOdometryIntegratedX[i + 1] = x[0] + self.deltaOdometryIntegratedX[i]
            self.deltaOdometryIntegratedY[i + 1] = x[1] + self.deltaOdometryIntegratedY[i]
            self.deltaOdometryIntegratedZ[i + 1] = x[2] + self.deltaOdometryIntegratedZ[i]
            
            q  = Quaternion(qw[i], qx[i], qy[i], qz[i])
            q0 = q*q0

    def compute_tokamak(self):

        try:
            self.dataTokamak.parse_metadata(self.tokamakDataFormatFilename,   self.tokamakDataFilename)
        except:
            print("No tokamak data")
            return

        self.tokamakTime = self.dataTokamak.get_nparray('child_time')
        self.tokamakX    = self.dataTokamak.get_nparray('x')
        self.tokamakY    = self.dataTokamak.get_nparray('y')
        self.tokamakZ    = self.dataTokamak.get_nparray('z')

        self.tokamakPeriod = self.tokamakTime[1:] - self.tokamakTime[:-1]
        dx = self.tokamakX[1:] - self.tokamakX[:-1]
        dy = self.tokamakY[1:] - self.tokamakY[:-1]
        dz = self.tokamakZ[1:] - self.tokamakZ[:-1]

        self.tokamakSpeed = np.nan_to_num(np.sqrt(dx*dx + dy*dy + dz*dz) / self.tokamakPeriod)

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

    def compute_velodyne(self):

        try:
            self.dataVelodyne.parse_metadata(self.velodyneDataFormatFilename,   self.velodyneDataFilename)
        except:
            print("No velodyne data")
            return

        self.velodyneCloudTime = self.dataVelodyne.get_nparray('cloud_time')
        self.velodynePeriod = self.velodyneCloudTime[1:] - self.velodyneCloudTime[:-1]
        self.velodyneDesync = self.dataVelodyne.get_nparray('pose_fixed_robot__child_time') - self.velodyneCloudTime

        files = os.listdir(self.velodynePCDPath)
        filenames = []
        for filename in files:
            if filename.endswith(".pcd"):
                filenames.append(filename)
        filenames.sort()

        pointCount = []
        for filename in filenames:

            try:
                pcdHeadLines = get_pcd_header(self.velodynePCDPath + filename).split("\n")
                if not any(["POINTS" in line.split(" ")[0] for line in pcdHeadLines]):
                    raise Exception("No POINTS metadata in \"", os.path.join(self.velodynePCDPath, filename), "\" file header")
                for line in pcdHeadLines:
                    words = line.split(" ")
                    if "POINTS" in words[0]:
                        pointCount.append(float(words[1]))
                
            except Exception as e:
                print("Could not get pcd file header : ", e) 
                pointCount.append(0)
        
        self.velodyneNbPoints = np.array(pointCount)
        self.velodyneMinTime  = self.velodyneCloudTime[0]
        self.hasVelodyneData = True

    def start_mission_time(self):

        times = []
        if self.frontMinTime > 0:
            times.append(self.frontMinTime)
        if self.rearMinTime > 0:
            times.append(self.rearMinTime)
        if self.navMinTime > 0:
            times.append(self.navMinTime)
        if self.velodyneMinTime > 0:
            times.append(self.velodyneMinTime)

        return min(times)

        
