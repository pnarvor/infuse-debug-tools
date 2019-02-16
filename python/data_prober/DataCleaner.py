import os
import numpy as np
from Metadata import Metadata

class DataCleaner:
    
    def __init__(self, dataRootDir):

        self.dataRootDir = dataRootDir

        # Front cam raw files
        self.frontLeftDataFormatFilename           = "front_cam/left/left_dataformat.txt"
        self.frontLeftDataFilename                 = "front_cam/left/left_all_metadata.txt"
        self.frontLeftIntegrityDataFormatFilename  = "front_cam/left/image_integrity_dataformat.txt"
        self.frontLeftIntegrityDataFilename        = "front_cam/left/image_integrity.txt"
        self.frontRightDataFormatFilename          = "front_cam/right/right_dataformat.txt"
        self.frontRightDataFilename                = "front_cam/right/right_all_metadata.txt"
        self.frontRightIntegrityDataFormatFilename = "front_cam/right/image_integrity_dataformat.txt"
        self.frontRightIntegrityDataFilename       = "front_cam/right/image_integrity.txt"
        self.frontDisparityDataFormatFilename      = "stereo/front_disparity/disparity_dataformat.txt"
        self.frontDisparityDataFilename            = "stereo/front_disparity/disparity_all_metadata.txt"

        # Rear cam raw files
        self.rearLeftDataFormatFilename           = "rear_cam/left/left_dataformat.txt"
        self.rearLeftDataFilename                 = "rear_cam/left/left_all_metadata.txt"
        self.rearLeftIntegrityDataFormatFilename  = "rear_cam/left/image_integrity_dataformat.txt"
        self.rearLeftIntegrityDataFilename        = "rear_cam/left/image_integrity.txt"
        self.rearRightDataFormatFilename          = "rear_cam/right/right_dataformat.txt"
        self.rearRightDataFilename                = "rear_cam/right/right_all_metadata.txt"
        self.rearRightIntegrityDataFormatFilename = "rear_cam/right/image_integrity_dataformat.txt"
        self.rearRightIntegrityDataFilename       = "rear_cam/right/image_integrity.txt"
        self.rearDisparityDataFormatFilename      = "stereo/rear_disparity/disparity_dataformat.txt"
        self.rearDisparityDataFilename            = "stereo/rear_disparity/disparity_all_metadata.txt"

        # Nav cam raw files
        self.navLeftDataFormatFilename           = "nav_cam/left/left_dataformat.txt"
        self.navLeftDataFilename                 = "nav_cam/left/left_all_metadata.txt"
        self.navLeftIntegrityDataFormatFilename  = "nav_cam/left/image_integrity_dataformat.txt"
        self.navLeftIntegrityDataFilename        = "nav_cam/left/image_integrity.txt"
        self.navRightDataFormatFilename          = "nav_cam/right/right_dataformat.txt"
        self.navRightDataFilename                = "nav_cam/right/right_all_metadata.txt"
        self.navRightIntegrityDataFormatFilename = "nav_cam/right/image_integrity_dataformat.txt"
        self.navRightIntegrityDataFilename       = "nav_cam/right/image_integrity.txt"
        self.navDisparityDataFormatFilename      = "stereo/nav_disparity/disparity_dataformat.txt"
        self.navDisparityDataFilename            = "stereo/nav_disparity/disparity_all_metadata.txt"

        #GPS raw files
        self.gpsDataFormatFilename      = "gps/gps_pose_info_dataformat.txt"
        self.gpsDataFilename            = "gps/gps_pose_info.txt"
        #Odometry raw files
        self.odometryDataFormatFilename = "odometry/dataformat.txt"
        self.odometryDataFilename       = "odometry/odometry.txt"
        #Tokamak raw files
        self.tokamakDataFormatFilename  = "tokamak/dataformat.txt"
        self.tokamakDataFilename        = "tokamak/tokamak.txt"

        #Velodyne raw files
        self.velodyneDataFormatFilename  = "velodyne/dataformat.txt"
        self.velodyneDataFilename        = "velodyne/all_metadata.txt"
        self.velodynePCDPath             = "velodyne/data/"

        self.dataFrontLeft           = Metadata()
        self.dataFrontRight          = Metadata()
        self.dataFrontDisparity      = Metadata()
        self.dataFrontLeftIntegrity  = Metadata()
        self.dataFrontRightIntegrity = Metadata()
        self.frontStereoDesync       = np.empty([0])
        self.frontStereoStamp        = np.empty([0])
        self.frontMinTime            = -1
        self.frontDisparityScore     = np.empty([0])
        self.frontLeftIntegrity      = np.empty([0])
        self.frontRightIntegrity     = np.empty([0])

        self.dataRearLeft           = Metadata()
        self.dataRearRight          = Metadata()
        self.dataRearDisparity      = Metadata()
        self.dataRearLeftIntegrity  = Metadata()
        self.dataRearRightIntegrity = Metadata()
        self.rearStereoDesync       = np.empty([0])
        self.rearStereoStamp        = np.empty([0])
        self.rearMinTime            = -1
        self.rearDisparityScore     = np.empty([0])
        self.rearLeftIntegrity      = np.empty([0])
        self.rearRightIntegrity     = np.empty([0])

        self.dataNavLeft           = Metadata()
        self.dataNavRight          = Metadata()
        self.dataNavDisparity      = Metadata()
        self.dataNavLeftIntegrity  = Metadata()
        self.dataNavRightIntegrity = Metadata()
        self.navStereoDesync       = np.empty([0])
        self.navStereoStamp        = np.empty([0])
        self.navMinTime            = -1
        self.navDisparityScore     = np.empty([0])
        self.navLeftIntegrity      = np.empty([0])
        self.navRightIntegrity     = np.empty([0])
        self.navTotalScore         = np.empty([0])

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

        # Tokamak
        self.dataTokamak   = Metadata()
        self.tokamakTime   = np.empty([0])
        self.tokamakX      = np.empty([0])
        self.tokamakY      = np.empty([0])
        self.tokamakZ      = np.empty([0])
        self.tokamakPeriod = np.empty([0])
        self.tokamakSpeed  = np.empty([0])

        # Velodyne
        self.dataVelodyne      = Metadata()
        self.velodyneCloudTime = np.empty([0])
        self.velodynePeriod    = np.empty([0])
        self.velodyneDesync    = np.empty([0])
        self.velodyneNbPoints  = np.empty([0])

    # Stereo desync #################################### 
    def compute_front_stereo_desync(self):

        self.dataFrontLeft.parse_metadata(self.dataRootDir + self.frontLeftDataFormatFilename,   self.dataRootDir + self.frontLeftDataFilename)
        self.dataFrontRight.parse_metadata(self.dataRootDir + self.frontRightDataFormatFilename,   self.dataRootDir + self.frontRightDataFilename)
        self.frontStereoDesync = np.abs(self.dataFrontRight.get_nparray('timestamp') - self.dataFrontLeft.get_nparray('timestamp'))
        self.frontStereoStamps = self.dataFrontLeft.get_nparray('timestamp')
        self.frontMinTime = self.frontStereoStamps[0]
        
    def compute_rear_stereo_desync(self):

        self.dataRearLeft.parse_metadata(self.dataRootDir + self.rearLeftDataFormatFilename,   self.dataRootDir + self.rearLeftDataFilename)
        self.dataRearRight.parse_metadata(self.dataRootDir + self.rearRightDataFormatFilename,   self.dataRootDir + self.rearRightDataFilename)
        self.rearStereoDesync = np.abs(self.dataRearRight.get_nparray('timestamp') - self.dataRearLeft.get_nparray('timestamp'))
        self.rearStereoStamps = self.dataRearLeft.get_nparray('timestamp')
        self.rearMinTime = self.rearStereoStamps[0]

    def compute_nav_stereo_desync(self):

        self.dataNavLeft.parse_metadata(self.dataRootDir + self.navLeftDataFormatFilename,   self.dataRootDir + self.navLeftDataFilename)
        self.dataNavRight.parse_metadata(self.dataRootDir + self.navRightDataFormatFilename,   self.dataRootDir + self.navRightDataFilename)
        self.navStereoDesync = np.abs(self.dataNavRight.get_nparray('timestamp') - self.dataNavLeft.get_nparray('timestamp'))
        self.navStereoStamps = self.dataNavLeft.get_nparray('timestamp')
        self.navMinTime = self.navStereoStamps[0]
       
    # Stereo desync #################################### 
    def compute_front_disparity_score(self):

        self.dataFrontDisparity.parse_metadata(self.dataRootDir + self.frontDisparityDataFormatFilename, self.dataRootDir + self.frontDisparityDataFilename)
        self.frontDisparityScore = 100 - self.dataFrontDisparity.get_nparray('percentage_of_paired_pixels')
        self.frontDisparityScore = self.frontDisparityScore - np.amin(self.frontDisparityScore)

    def compute_rear_disparity_score(self):

        self.dataRearDisparity.parse_metadata(self.dataRootDir + self.rearDisparityDataFormatFilename, self.dataRootDir + self.rearDisparityDataFilename)
        self.rearDisparityScore = 100 - self.dataRearDisparity.get_nparray('percentage_of_paired_pixels')
        self.rearDisparityScore = self.rearDisparityScore - np.amin(self.rearDisparityScore)

    def compute_nav_disparity_score(self):

        self.dataNavDisparity.parse_metadata(self.dataRootDir + self.navDisparityDataFormatFilename, self.dataRootDir + self.navDisparityDataFilename)
        self.navDisparityScore = 100 - self.dataNavDisparity.get_nparray('percentage_of_paired_pixels')
        self.navDisparityScore = self.navDisparityScore - np.amin(self.navDisparityScore)

    # Image integrity
    def compute_front_integrity(self):

        self.dataFrontLeftIntegrity.parse_metadata(self.dataRootDir + self.frontLeftIntegrityDataFormatFilename, self.dataRootDir + self.frontLeftIntegrityDataFilename)
        self.dataFrontRightIntegrity.parse_metadata(self.dataRootDir + self.frontRightIntegrityDataFormatFilename, self.dataRootDir + self.frontRightIntegrityDataFilename)
        self.frontLeftIntegrity = self.dataFrontLeftIntegrity.get_nparray('score')
        self.frontRightIntegrity = self.dataFrontRightIntegrity.get_nparray('score')

    def compute_rear_integrity(self):

        self.dataRearLeftIntegrity.parse_metadata(self.dataRootDir + self.rearLeftIntegrityDataFormatFilename, self.dataRootDir + self.rearLeftIntegrityDataFilename)
        self.dataRearRightIntegrity.parse_metadata(self.dataRootDir + self.rearRightIntegrityDataFormatFilename, self.dataRootDir + self.rearRightIntegrityDataFilename)
        self.rearLeftIntegrity = self.dataRearLeftIntegrity.get_nparray('score')
        self.rearRightIntegrity = self.dataRearRightIntegrity.get_nparray('score')

    def compute_nav_integrity(self):

        self.dataNavLeftIntegrity.parse_metadata(self.dataRootDir + self.navLeftIntegrityDataFormatFilename, self.dataRootDir + self.navLeftIntegrityDataFilename)
        self.dataNavRightIntegrity.parse_metadata(self.dataRootDir + self.navRightIntegrityDataFormatFilename, self.dataRootDir + self.navRightIntegrityDataFilename)
        self.navLeftIntegrity = self.dataNavLeftIntegrity.get_nparray('score')
        self.navLeftIntegrity = self.navLeftIntegrity - np.amin(self.dataNavLeftIntegrity.get_nparray('score'))
        self.navRightIntegrity = self.dataNavRightIntegrity.get_nparray('score')
        self.navRightIntegrity = self.navRightIntegrity - np.amin(self.dataNavRightIntegrity.get_nparray('score'))

    # Stereo pair total score
    def compute_front_total_score(self):

        dataArray = np.vstack((self.frontStereoDesync, self.frontDisparityScore, self.frontLeftIntegrity, self.frontRightIntegrity))

        sig = np.zeros([4,4])
        sig[0,0] = 1.0 / np.dot(dataArray[0,:], dataArray[0,:])
        sig[1,1] = 1.0 / np.dot(dataArray[1,:], dataArray[1,:])
        sig[2,2] = 1.0 / np.dot(dataArray[2,:], dataArray[2,:])
        sig[3,3] = 1.0 / np.dot(dataArray[3,:], dataArray[3,:])

        self.frontTotalScore = 1 - np.exp(-np.sum(np.multiply(dataArray, sig @ dataArray), axis=0))

    def compute_rear_total_score(self):

        dataArray = np.vstack((self.rearStereoDesync, self.rearDisparityScore, self.rearLeftIntegrity, self.rearRightIntegrity))

        sig = np.zeros([4,4])
        sig[0,0] = 1.0 / np.dot(dataArray[0,:], dataArray[0,:])
        sig[1,1] = 1.0 / np.dot(dataArray[1,:], dataArray[1,:])
        sig[2,2] = 1.0 / np.dot(dataArray[2,:], dataArray[2,:])
        sig[3,3] = 1.0 / np.dot(dataArray[3,:], dataArray[3,:])

        self.rearTotalScore = 1 - np.exp(-np.sum(np.multiply(dataArray, sig @ dataArray), axis=0))

    def compute_nav_total_score(self):

        dataArray = np.vstack((self.navStereoDesync, self.navDisparityScore, self.navLeftIntegrity, self.navRightIntegrity))

        sig = np.zeros([4,4])
        sig[0,0] = 1.0 / np.dot(dataArray[0,:], dataArray[0,:])
        sig[1,1] = 1.0 / np.dot(dataArray[1,:], dataArray[1,:])
        sig[2,2] = 1.0 / np.dot(dataArray[2,:], dataArray[2,:])
        sig[3,3] = 1.0 / np.dot(dataArray[3,:], dataArray[3,:])

        self.navTotalScore = 1 - np.exp(-np.sum(np.multiply(dataArray, sig @ dataArray), axis=0))

    def compute_gps(self):

        self.dataGPS.parse_metadata(self.dataRootDir + self.gpsDataFormatFilename,   self.dataRootDir + self.gpsDataFilename)

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

        self.dataOdometry.parse_metadata(self.dataRootDir + self.odometryDataFormatFilename,   self.dataRootDir + self.odometryDataFilename)

        self.odometryTime = self.dataOdometry.get_nparray('child_time')
        self.odometryX    = self.dataOdometry.get_nparray('x')
        self.odometryY    = self.dataOdometry.get_nparray('y')
        self.odometryZ    = self.dataOdometry.get_nparray('z')

        self.odometryPeriod = self.odometryTime[1:] - self.odometryTime[:-1]
        dx = self.odometryX[1:] - self.odometryX[:-1]
        dy = self.odometryY[1:] - self.odometryY[:-1]
        dz = self.odometryZ[1:] - self.odometryZ[:-1]

        self.odometrySpeed = np.nan_to_num(np.sqrt(dx*dx + dy*dy + dz*dz) / self.odometryPeriod)

    def compute_tokamak(self):

        self.dataTokamak.parse_metadata(self.dataRootDir + self.tokamakDataFormatFilename,   self.dataRootDir + self.tokamakDataFilename)

        self.tokamakTime = self.dataTokamak.get_nparray('child_time')
        self.tokamakX    = self.dataTokamak.get_nparray('x')
        self.tokamakY    = self.dataTokamak.get_nparray('y')
        self.tokamakZ    = self.dataTokamak.get_nparray('z')

        self.tokamakPeriod = self.tokamakTime[1:] - self.tokamakTime[:-1]
        dx = self.tokamakX[1:] - self.tokamakX[:-1]
        dy = self.tokamakY[1:] - self.tokamakY[:-1]
        dz = self.tokamakZ[1:] - self.tokamakZ[:-1]

        self.tokamakSpeed = np.nan_to_num(np.sqrt(dx*dx + dy*dy + dz*dz) / self.tokamakPeriod)

    def compute_velodyne(self):

        self.dataVelodyne.parse_metadata(self.dataRootDir + self.velodyneDataFormatFilename,   self.dataRootDir + self.velodyneDataFilename)

        self.velodyneCloudTime = self.dataVelodyne.get_nparray('cloud_time')
        self.velodynePeriod = self.velodyneCloudTime[1:] - self.velodyneCloudTime[:-1]
        self.velodyneDesync = self.dataVelodyne.get_nparray('pose_fixed_robot__child_time') - self.velodyneCloudTime

        files = os.listdir(self.dataRootDir + self.velodynePCDPath)
        filenames = []
        for filename in files:
            if filename.endswith(".pcd"):
                filenames.append(filename)
        filenames.sort()

        pointCount = []
        for filename in filenames:
            pcdFile = open(self.dataRootDir + self.velodynePCDPath + filename, "r")
            lineCount = 0
            # while lineCount < 10:
            while lineCount < 9:
                line = pcdFile.readline()
                print(line)
                if line.split(" ")[0] == "POINTS":
                    pointCount.append(int(line.split(" ")[1]))
                    break
                lineCount += 1
            if lineCount >= 9:
                print("Error reading " + self.dataRootDir + self.velodynePCDPath + filename + ". File corrupted.\n")
                pointCount.append(0)
        
        self.velodyneNbPoints = np.array(pointCount)

    def start_mission_time(self):

        times = []
        if self.frontMinTime > 0:
            times.append(self.frontMinTime)
        if self.rearMinTime > 0:
            times.append(self.rearMinTime)
        if self.navMinTime > 0:
            times.append(self.navMinTime)

        return min(times)
        
