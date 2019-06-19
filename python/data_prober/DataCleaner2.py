import os
import numpy as np
import matplotlib.pyplot as plt
import io
from pyquaternion import Quaternion
from scipy.signal import medfilt
import yaml
from shutil import copyfile

from .Utils         import add_twiny
from .Utils         import InfuseTransform
from .Utils         import extrinsic_synchro
from .Metadata      import Metadata
from .RobotPoseData import RobotPoseData
from .VelodyneData  import VelodyneData
from .CameraData    import CameraData

from .ExportedVelodyneData import ExportedVelodyneData
from .ExportedCameraData   import ExportedCameraData

class DataCleaner2:
    
    def __init__(self, dataRootDir, exportPath=""):

        self.dataRootDir        = dataRootDir
        self.exportPath         = exportPath
        self.exportPlanFilename = os.path.join(dataRootDir, "export_plan.yaml")

        self.sensors = ['velodyne', 'nav_cam', 'front_cam', 'rear_cam', 'pano_cam']
        # self.sensors = ['velodyne', 'nav_cam', 'front_cam', 'rear_cam']

        self.robotPoseData = RobotPoseData(dataRootDir)
        # self.velodyneData  = VelodyneData(dataRootDir, exportPath)
        # self.navData       = CameraData(dataRootDir, "nav"  , exportPath)
        # self.frontData     = CameraData(dataRootDir, "front", exportPath)
        # self.rearData      = CameraData(dataRootDir, "rear" , exportPath)
        self.sensorData              = {}
        self.sensorData['velodyne' ] = VelodyneData(dataRootDir, exportPath)
        self.sensorData['nav_cam'  ] = CameraData(dataRootDir, "nav"  , exportPath)
        self.sensorData['front_cam'] = CameraData(dataRootDir, "front", exportPath)
        self.sensorData['rear_cam' ] = CameraData(dataRootDir, "rear" , exportPath)
        self.sensorData['pano_cam' ] = CameraData(dataRootDir, "pano" , exportPath)

        # Global data
        self.minTime       = -1

    def load(self):

        self.robotPoseData.load()

        keysToRemove = []
        for sensor in self.sensors:
            self.sensorData[sensor].load()
            if not self.sensorData[sensor].filesLoaded:
                keysToRemove.append(sensor)

        # removing data we were unable to load
        for key in keysToRemove:
            del self.sensorData[key]
        self.sensors = self.sensorData.keys()

        self.compute_mission_time()
        for sensor in self.sensors:
            self.sensorData[sensor].compute_retagged_poses(self.robotPoseData)
            self.sensorData[sensor].tag_odometry(self.robotPoseData)
            self.sensorData[sensor].suggest_broken_data()

    def export(self):

        brokenData = {}
        exportPlan = yaml.safe_load(open(self.exportPlanFilename, "r"))
        if 'data_to_remove' in exportPlan.keys():
            for sensor in self.sensors:
                if sensor in exportPlan['data_to_remove'].keys():
                    brokenData[sensor] = exportPlan['data_to_remove'][sensor]

        for setName in exportPlan['datasets_to_export'].keys():
            timeInterval = exportPlan['datasets_to_export'][setName]['time_interval']
            sensors      = exportPlan['datasets_to_export'][setName]['sensors']

            exporters = {}
            sensorToExport = []
            if 'velodyne' in sensors and 'velodyne' in self.sensors:
                sensorToExport.append('velodyne')
                exporters['velodyne']  = ExportedVelodyneData(self.sensorData['velodyne'],
                                                              brokenData['velodyne'])
            if 'nav_cam' in sensors and 'nav_cam' in self.sensors:
                sensorToExport.append('nav_cam')
                exporters['nav_cam']   = ExportedCameraData(self.sensorData['nav_cam'],
                                                            brokenData['nav_cam'])
            if 'front_cam' in sensors and 'front_cam' in self.sensors:
                sensorToExport.append('front_cam')
                exporters['front_cam'] = ExportedCameraData(self.sensorData['front_cam'],
                                                            brokenData['front_cam'])
            if 'rear_cam' in sensors and 'rear_cam' in self.sensors:
                sensorToExport.append('rear_cam')
                exporters['rear_cam']  = ExportedCameraData(self.sensorData['rear_cam'],
                                                            brokenData['rear_cam'])
            if 'pano_cam' in sensors and 'pano_cam' in self.sensors:
                sensorToExport.append('pano_cam')
                exporters['pano_cam']  = ExportedCameraData(self.sensorData['pano_cam'],
                                                            brokenData['pano_cam'])
            sensors = sensorToExport
            for sensor in exporters.keys():
                exporters[sensor].clean_data()

            self.synchronize_cameras(exporters, tol=75)

            # finding a common time for mission begin
            timeInterval = [1000000.0 * timeInterval[0]  + self.minTime,
                            1000000.0 * timeInterval[-1] + self.minTime]
            datasetT0 = self.compute_dataset_time(timeInterval[0],
                                                  [exporters[key] for key in sensors])
            try:
                os.makedirs(os.path.join(self.exportPath, setName))
            except:
                pass
            copyfile(self.exportPlanFilename,
                     os.path.join(self.exportPath, setName,
                                  os.path.split(self.exportPlanFilename)[1]))
            for sensor in sensors:
                outputPath = os.path.join(self.exportPath, setName, sensor)
                exporters[sensor].export(timeInterval, datasetT0, outputPath)


    def compute_mission_time(self):

        times = []
        for sensor in self.sensors:
            if self.sensorData[sensor].minTime > 0:
                times.append(self.sensorData[sensor].minTime)
        self.minTime = min(times)

        for sensor in self.sensors:
            self.sensorData[sensor].minTime = self.minTime
        self.robotPoseData.minTime = self.minTime

    def compute_dataset_time(self, t0, exporters):
        
        times = []
        for exp in exporters:
            times.append(exp.utcStamp[np.where(np.array(exp.utcStamp) >= t0)[0][0]])
        return min(times)

    def synchronize_cameras(self, exporters, tol=75):

        camExporters = {}
        stampsToSync = []
        for sensor in exporters.keys():
            if not 'cam' in sensor or 'pano' in sensor:
                continue
            stampsToSync.append(exporters[sensor].utcStamp)
            camExporters[sensor] = exporters[sensor]
        if len(stampsToSync) <= 1:
            return
        
        toDelete = extrinsic_synchro(stampsToSync, tolerance=tol)
        # print("Indexes to delete :\n")
        for key, toDel in zip(camExporters.keys(), toDelete):
            # print(" -- " + camExporters[key].cameraName + ", ", len(camExporters[key].utcStamp)," :\n", toDel)
            camExporters[key].dataToRemove = toDel
            camExporters[key].clean_data()

    def display(self, verbose=False, blocking=False):

        self.robotPoseData.display(verbose, blocking)
        for sensor in self.sensors:
            self.sensorData[sensor].display(verbose, blocking)

    def check_velodyne_exported(self, path):
        
        formatFilename = os.path.join(path, 'dataformat.txt')
        dataFilename   = os.path.join(path, 'all_metadata.txt')
        data = Metadata()
        data.parse_metadata(formatFilename, dataFilename)

        time_span = [data.data_time_stamp[0], data.data_time_stamp[1]]
        fig, axes = plt.subplots(3,1, sharex=True, sharey=False)
        axes[0].plot((np.array(data.data_time_stamp) - np.array(data.robot_to_world_pose_time)) / 1000.0, label="Desync cloud / pose")
        axes[0].legend(loc="upper right")
        axes[0].set_xlabel("Scan number")
        axes[0].set_ylabel("Desync (ms)")
        axes[0].grid()
        add_twiny(axes[0], time_span, label="Mission time (s)")
        axes[1].plot(data.cloud_number_of_points, label="Number of points / clouds")
        axes[1].legend(loc="lower right")
        axes[1].set_xlabel("Scan number")
        axes[1].set_ylabel("Number of points")
        axes[1].grid()
        add_twiny(axes[1], time_span, label="Mission time (s)")
        axes[2].plot(data.robot_to_world_pose_sig_x, label="Easting  sigma")
        axes[2].plot(data.robot_to_world_pose_sig_y, label="Northing sigma")
        axes[2].plot(data.robot_to_world_pose_sig_z, label="Height   sigma")
        axes[2].legend(loc="upper right")
        axes[2].set_xlabel("Scan number")
        axes[2].set_ylabel("GPS sigma (cm)")
        axes[2].grid()
        add_twiny(axes[2], time_span, label="Mission time (s)")

        fig, axes = plt.subplots(1,1, sharex=True, sharey=False)
        axes.plot(np.array(data.robot_to_world_pose_curvilinear_abs), label="Curvilinear abs")
        axes.legend(loc="upper right")
        axes.set_xlabel("Scan number")
        axes.set_ylabel("Curvilinear Abs (m)")
        axes.grid()
        add_twiny(axes, time_span, label="Mission time (s)")

        plt.show(block=False)
