import os
import numpy             as np
import transformations   as tr
import matplotlib.pyplot as plt
import copy              as cp

from pyquaternion import Quaternion
from scipy.signal import medfilt
from statistics   import median

class EulerAngles: # fixed convention defined by conversion functions

    eulerConvention = 'sxyz'

    def __init__(self, roll = 0.0, pitch=0.0, yaw=0.0):
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw

    def from_quaternion(q):
        angles = tr.euler_from_quaternion(np.array([q[0],q[1],q[2],q[3]]),
                                          EulerAngles.eulerConvention)
        return EulerAngles(angles[0], angles[1], angles[2])

    def to_quaternion(self):
        return Quaternion(tr.quaternion_from_euler(roll,
                                                   pitch,
                                                   yaw,
                                                   EulerAngles.eulerConvention))

def create_folder(path):

    if os.path.exists(path):
        raise Exception("Failed to create folder : \"" + path
                        + "\" already exists !")
    os.makedirs(path)

def spike_detector(data):
   return np.abs(data - medfilt(data, kernel_size=3)) 

class InfuseTransform:

    def __init__(self, translation = np.array([0.0, 0.0, 0.0]),
                 orientation = Quaternion()):
        self.translation = translation
        self.orientation = orientation

    def __str__(self):
        return (   "translation : " + str(self.translation)
               + "\norientation : " + str(self.orientation))

    def __mul__(self, other):

        if isinstance(other, InfuseTransform):
            return InfuseTransform(
                self.orientation.rotate(other.translation) + self.translation,
                self.orientation * other.orientation)
        elif isinstance(other, np.array):
            return self.orientation.rotate(other.translation)+self.translation
        else:
            raise Exception("Type not handled by __mul__")

    def inverse(self):

        return InfuseTransform(-(self.orientation.inverse.rotate(self.translation)),
                               self.orientation.inverse)

    def get_euler(self):
        return EulerAngles.from_quaternion(self.orientation)

def add_twiny(ax0, newSpan, label=""):
    
    ax1 = ax0.twiny()
    ax1.set_xlim(newSpan[0], newSpan[-1])
    ax1.set_xlabel(label)

def plot_highlighted(ax, yValues, xValues=[], highlighted=[], label=""):

    if len(xValues) == 0:
        ax.plot(yValues, label=label)
        if len(highlighted) > 0:
            ax.plot(highlighted, [yValues[i] for i in  highlighted], 'or')
    else:
        ax.plot(xValues, yValues, label=label)
        if len(highlighted) > 0:
            ax.plot([xValues[i] for i in  highlighted], [yValues[i] for i in  highlighted], 'or')

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
 
# remove stereo pairs to keep sync between stereo benches, tolerance is expressed in milliseconds
def extrinsic_synchro(stamps, tolerance = 75):

    def are_synched(stamps, tol):
        m = median(stamps)
        if any([abs(stamp - m) > tol for stamp in stamps]):
            # l = [(t - m)/1000.0 for t in stamps]
            # print(l)
            return False
        else:
            return True

    stamps            = cp.deepcopy(stamps)
    currentStampIndex = [0  for stampList in stamps]
    toDelete          = [[] for stampList in stamps]
    print("Stereo benches extrinsic synchronization.")
    tolerance = 1000*tolerance

    while not any([len(stampList) <= 0 for stampList in stamps]):

        currentStamps = [stampList[0] for stampList in stamps]
        if not are_synched(currentStamps, tolerance):
            minIndex = currentStamps.index(min(currentStamps))
            toDelete[minIndex].append(currentStampIndex[minIndex])
            currentStampIndex[minIndex] = currentStampIndex[minIndex] + 1
            del(stamps[minIndex][0])
            continue

        # Removing first stamps when synchronized
        for stampList in stamps:
            del(stampList[0])
        currentStampIndex = [i + 1 for i in currentStampIndex]

    # Must delete remaining images to have same number of images
    for index, stampList, delList in zip(currentStampIndex, stamps, toDelete):
        for i in range(len(stampList)):
            delList.append(index + i)

    return toDelete
