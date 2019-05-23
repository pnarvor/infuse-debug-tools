import os
import numpy as np
import io
from pyquaternion import Quaternion
from scipy.signal import medfilt
import yaml

from .Utils         import InfuseTransform
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

            # finding a common time for mission begin
            timeInterval = [1000000.0 * timeInterval[0]  + self.minTime,
                            1000000.0 * timeInterval[-1] + self.minTime]
            datasetT0 = self.compute_dataset_time(timeInterval[0],
                                                  [exporters[key] for key in sensors])
            print(datasetT0)
            for sensor in sensors:
                outputPath = os.path.join(self.exportPath, setName, sensor)
                exporters[sensor].export(timeInterval, datasetT0, outputPath)

        # self.velodyneData.export()
        # self.navData.export()
        # self.frontData.export()
        # self.rearData.export()

    def compute_mission_time(self):

        times = []
        for sensor in self.sensors:
            if self.sensorData[sensor].minTime > 0:
                times.append(self.sensorData[sensor].minTime)
        self.minTime = min(times)

        for sensor in self.sensors:
            self.sensorData[sensor].minTime = self.minTime

    def compute_dataset_time(self, t0, exporters):
        
        times = []
        for exp in exporters:
            times.append(exp.utcStamp[np.where(np.array(exp.utcStamp) >= t0)[0][0]])
        return min(times)

    def display(self, verbose=False, blocking=False):

        self.robotPoseData.display(verbose, blocking)
        for sensor in self.sensors:
            self.sensorData[sensor].display(verbose, blocking)
