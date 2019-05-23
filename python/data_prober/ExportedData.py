import os
import numpy as np
import matplotlib.pyplot as plt
import io
from pyquaternion import Quaternion
import yaml
import progressbar
from shutil import copyfile

from .Utils       import InfuseTransform
from .Utils       import create_folder
from .Metadata    import Metadata
from .DataCleaner import spike_detector

class ExportedData:

    # def __init__(self, exportPlanFilename="", exportPath=""):

    #     # export plan set in a .yaml file
    #     self.exportPath         = exportPath
    #     self.exportPlanFilename = exportPlanFilename
    #     self.dataPaths          = [] # set by superclass 
    #     self.dataExportSubPaths = [] # set by superclass
    #     self.intervalsToExport  = {} # keys = export name, items = intervals in time ???
    #     self.dataToRemove       = [] # list of data points to remove

    #     # Some export filenames
    #     self.local_frame_file   = "reference_frame.yaml"
    #    
    #     # filled in data
    #     # self.startTime    = []
    #     self.utcStamp     = []
    #     self.dataIndex    = []

    #     self.ltfPose      = []
    #     self.ltfPoseTime  = [] # to keep ?
    #     self.ltfCurvAbs   = []
    #     self.gpsStddev    = [] 

    #     self.odoAbsPose   = []
    #     self.odoPoseTime  = [] # to keep ?
    #     self.odoCurvAbs   = []

    #     self.sensorPose   = []
    #     self.ltfToGtf     = None

    #     self.ltfSpeed     = []
    #     self.odoSpeed     = []

    #     # computed data
    #     self.odoDeltaPose = []
    #     self.sensorWorld  = []

    #     # Metadata struct
    #     self.metadata     = Metadata()
    #     self.minTime      = -1

    def __init__(self, dataRootDir, dataToRemove):

        self.dataRootDir      = dataRootDir
        self.dataToRemove     = dataToRemove
        self.local_frame_file = "reference_frame.yaml"
        
    # def export(self):
    #    
    #     # self.parse_export_plan()
    #     # self.clean_data()
    #     for dataSetName in self.intervalsToExport.keys():

    #         # all magic starts here
    #         interval = self.time_to_index(self.intervalsToExport[dataSetName], self.utcStamp)
    #         self.build_metadata_struct(interval)

    #         outputPath = os.path.join(self.exportPath, dataSetName)
    #         create_folder(outputPath)
    #         self.metadata.write_metadata_files(outputPath)
    #         print("Data copy is commented for debug")
    #         # self.copy_data(outputPath, interval)
    #         self.write_local_frame_file(os.path.join(outputPath, "reference_frame.yaml"))
    #         self.write_odo_frame_file(os.path.join(outputPath, "start_position.yaml"))

    def export(self, interval, t0, outputPath):

        interval = self.time_to_index(interval, self.utcStamp)
        self.build_metadata_struct(interval, t0)
        create_folder(outputPath)
        self.metadata.write_metadata_files(outputPath)
        # print("Data copy is commented for debug")
        self.copy_data(outputPath, interval)
        self.write_local_frame_file(os.path.join(outputPath, "reference_frame.yaml"))
        self.write_odo_frame_file(os.path.join(outputPath, "start_position.yaml"))

    def time_to_index(self, timeInterval, stamps):

        # indexInterval = []
        # if timeInterval[0] == 0:
        #     indexInterval.append(0)
        # elif timeInterval[0] > stamps[-1]:
        #     return []
        # else:
        #     indexInterval.append(np.where(np.array(stamps) >= timeInterval[0])[0][0])
        # if timeInterval[-1] == -1:
        #     indexInterval.append(len(stamps)-1)
        # elif timeInterval[-1] < stamps[0]:
        #     return []
        # else:
        #     indexInterval.append(np.where(np.array(stamps) <= timeInterval[-1])[0][-1])
        # return indexInterval
        indexInterval = []
        if timeInterval[0] == 0:
            indexInterval.append(0)
        elif timeInterval[0] > stamps[-1]:
            return []
        else:
            indexInterval.append(np.where(np.array(stamps) >= timeInterval[0])[0][0])
        if timeInterval[-1] == -1:
            indexInterval.append(len(stamps)-1)
        elif timeInterval[-1] < stamps[0]:
            return []
        else:
            indexInterval.append(np.where(np.array(stamps) <= timeInterval[-1])[0][-1])
        return indexInterval

    def parse_export_plan(self):
    
        f = open(self.exportPlanFilename, 'r')
        exportPlan = yaml.safe_load(f)
        if not "intervals_to_export" in exportPlan.keys():
            raise Exception("No intervals_to_export in export plan file")
        self.intervalsToExport = exportPlan['intervals_to_export']
        if not "data_to_remove" in exportPlan.keys():
            print("Warning : No data to remove in export plan"
                  + "Have you got a perfect dataset ?! (I don't believe you.)")
            return
        if not isinstance(exportPlan['data_to_remove'], list):
            self.dataToRemove = [exportPlan['data_to_remove']]
        else:
            self.dataToRemove = exportPlan['data_to_remove']

        print("Data to remove : ",  self.dataToRemove)
        print("Sets to export :\n", self.intervalsToExport)

        for key in self.intervalsToExport:
            self.intervalsToExport[key] = (1000000.0*np.array(self.intervalsToExport[key]) + self.minTime).tolist()
        
    def clean_data(self):

        print("Clean data poses : ", self.dataToRemove)
        self.dataIndex = [i for i in range(len(self.utcStamp))]

        for index in reversed(self.dataToRemove):

            self.dataIndex.pop(index)
            # self.startTime.pop(index)
            self.utcStamp.pop(index)
            self.ltfPose.pop(index)
            self.ltfPoseTime.pop(index)
            self.ltfCurvAbs.pop(index)
            self.gpsStddev.pop(index)
            self.odoAbsPose.pop(index)
            self.odoPoseTime.pop(index)
            self.odoCurvAbs.pop(index)
            self.sensorPose.pop(index)

            self.ltfSpeed.pop(index)
            self.odoSpeed.pop(index)

            # # removing elements is modifing the intervals ot export
            # for inter in self.intervalsToExport.values():
            #     if index < inter[0]:
            #         inter[0] = inter[0] - 1
            #     if index <= inter[-1]:
            #         inter[-1] = inter[-1] - 1

        self.compute_delta_odometry()

    def compute_delta_odometry(self):

        self.odoDeltaPose = [InfuseTransform()]
        lastPose = self.odoDeltaPose[0]
        for pose in self.odoAbsPose[1:]:
            translation = lastPose.orientation.inverse.rotate(
                                pose.translation - lastPose.translation)
            orientation = lastPose.orientation.inverse*pose.orientation
            self.odoDeltaPose.append(InfuseTransform(translation, orientation))
            lastPose = pose

    def add_metadata(self, name, data):
      
        print("Adding", str(len(data)), "data to export : \"" + name + "\"")
        self.metadata.add_field(name)
        self.metadata[name] = data;

    def build_metadata_struct(self, interval, t0):

        if len(interval) == 0:
            raise Exception("Nothing to export : issue with export plan ?")

        # t0 = self.utcStamp[interval[0]]
        # t0 = self.minTime
        # interval = self.time_to_index(interval, self.utcStamp)
        s = slice(interval[0], interval[-1] + 1)

        self.metadata = Metadata()

        self.add_metadata('index', [i for i in range(interval[-1] - interval[0] + 1)])

        self.add_metadata('data_time_stamp', [int(t-t0) for t in self.utcStamp[s]])
        self.add_metadata('data_utc_time', self.utcStamp[s])
        self.add_metadata('robot_to_world_pose_time', [int(t-t0) for t in self.ltfPoseTime[s]])
        self.add_metadata('robot_to_world_pose_x', [p.translation[0] for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_y', [p.translation[1] for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_z', [p.translation[2] for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_qw', [p.orientation[0] for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_qx', [p.orientation[1] for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_qy', [p.orientation[2] for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_qz', [p.orientation[3] for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_roll',  [p.get_euler().roll  for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_pitch', [p.get_euler().pitch for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_yaw',   [p.get_euler().yaw   for p in self.ltfPose[s]])
        self.add_metadata('robot_to_world_pose_sig_x', [sig[0] for sig in self.gpsStddev[s]])
        self.add_metadata('robot_to_world_pose_sig_y', [sig[1] for sig in self.gpsStddev[s]])
        self.add_metadata('robot_to_world_pose_sig_z', [sig[2] for sig in self.gpsStddev[s]])
        self.add_metadata('robot_to_world_pose_curvilinear_abs', [c - self.ltfCurvAbs[interval[0]] for c in self.ltfCurvAbs[s]])
        self.add_metadata('robot_to_world_speed', self.ltfSpeed[s])
        self.add_metadata('odometry_time', [int(t-t0) for t in self.odoPoseTime[s]])
        self.add_metadata('odometry_x', [p.translation[0] for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_y', [p.translation[1] for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_z', [p.translation[2] for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_qw', [p.orientation[0] for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_qx', [p.orientation[1] for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_qy', [p.orientation[2] for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_qz', [p.orientation[3] for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_roll',  [p.get_euler().roll  for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_pitch', [p.get_euler().pitch for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_yaw',   [p.get_euler().yaw   for p in self.odoDeltaPose[s]])
        self.add_metadata('odometry_curvilinear_abs', [c - self.odoCurvAbs[interval[0]] for c in self.odoCurvAbs[s]])
        self.add_metadata('odometry_speed', self.odoSpeed[s])
        self.add_metadata('sensor_to_robot_pose_x', [p.translation[0] for p in self.sensorPose[s]])
        self.add_metadata('sensor_to_robot_pose_y', [p.translation[1] for p in self.sensorPose[s]])
        self.add_metadata('sensor_to_robot_pose_z', [p.translation[2] for p in self.sensorPose[s]])
        self.add_metadata('sensor_to_robot_pose_qw',[p.orientation[0] for p in self.sensorPose[s]])
        self.add_metadata('sensor_to_robot_pose_qx',[p.orientation[1] for p in self.sensorPose[s]])
        self.add_metadata('sensor_to_robot_pose_qy',[p.orientation[2] for p in self.sensorPose[s]])
        self.add_metadata('sensor_to_robot_pose_qz',[p.orientation[3] for p in self.sensorPose[s]])
        self.add_metadata('sensor_to_robot_pose_roll',  [p.get_euler().roll  for p in self.sensorPose[s]])
        self.add_metadata('sensor_to_robot_pose_pitch', [p.get_euler().pitch for p in self.sensorPose[s]])
        self.add_metadata('sensor_to_robot_pose_yaw',   [p.get_euler().yaw   for p in self.sensorPose[s]])
# self.add_metadata('sensor_to_world_pose_x', [p.translation[0] for p in self.sensorWorld[s]])
# self.add_metadata('sensor_to_world_pose_y', [p.translation[1] for p in self.sensorWorld[s]])
# self.add_metadata('sensor_to_world_pose_z', [p.translation[2] for p in self.sensorWorld[s]])
# self.add_metadata('sensor_to_world_pose_qw',[p.orientation[0] for p in self.sensorWorld[s]])
# self.add_metadata('sensor_to_world_pose_qx',[p.orientation[1] for p in self.sensorWorld[s]])
# self.add_metadata('sensor_to_world_pose_qy',[p.orientation[2] for p in self.sensorWorld[s]])
# self.add_metadata('sensor_to_world_pose_qz',[p.orientation[3] for p in self.sensorWorld[s]])
        # print("Sets to export :\n", self.intervalsToExport)

    def copy_data(self, outputPath, interval):

        exportSubPaths = []
        for dataPath, expPath in zip(self.dataPaths, self.dataExportSubPaths):
            if not os.path.isdir(dataPath):
                raise Exception("Data path \"" + dataPath + "\" does not exists")
            exportSubPaths.append(os.path.join(outputPath, expPath))
            os.makedirs(os.path.join(outputPath, expPath))

        print("Copying data files. This may take a while.") 
        for i, dataIndex in zip(progressbar.progressbar(range(len(self.metadata.index))),
                                self.dataIndex[interval[0]:interval[-1]+1]):
            inputStr  = format(dataIndex, '05d')
            outputStr = format(i, '05d')
            for dataPath, expPath, ext in zip(self.dataPaths,
                                              exportSubPaths,
                                              self.dataExtensions):

                copyfile(os.path.join(dataPath, inputStr + ext),
                         os.path.join(expPath, outputStr + ext))
                # print("Copying", os.path.join(dataPath, inputStr + ext), "to",
                #                  os.path.join(expPath, outputStr + ext))

    def write_local_frame_file(self, path):
        f = open(path, 'w')
        f.write("# Origin of reference frame expressed in the UTM 30R tile\n")
        f.write("---\n")
        f.write("translation: [" + (str(self.ltfToGtf.translation[0]) + ", "
                                 +  str(self.ltfToGtf.translation[1]) + ", "
                                 +  str(self.ltfToGtf.translation[2]) + "]\n"))

    def write_odo_frame_file(self, path):
        f = open(path, 'w')
        f.write("# Starting position of the robot expressed in the local frame\n")
        f.write("# (Same pose as the first robot_to_world position)\n")
        f.write("# orientation in (qw, qx, qy, qz) format\n")
        f.write("---\n")
        f.write("translation: [" + (str(self.ltfPose[0].translation[0]) + ", "
                                 +  str(self.ltfPose[0].translation[1]) + ", "
                                 +  str(self.ltfPose[0].translation[2]) + "]\n"))
        f.write("orientation: [" + (str(self.ltfPose[0].orientation[0]) + ", "
                                 +  str(self.ltfPose[0].orientation[1]) + ", "
                                 +  str(self.ltfPose[0].orientation[2]) + ", "
                                 +  str(self.ltfPose[0].orientation[3]) + "]\n"))
