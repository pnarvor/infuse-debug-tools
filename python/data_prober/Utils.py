import numpy as np
import transformations as tr
from pyquaternion import Quaternion
from scipy.signal import medfilt

class EulerAngles: # fixed convention defined by conversion functions

    self.eulerConvention = 'sxyz'

    def __init__(roll = 0.0, pitch=0.0, yaw=0.0):
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw

    def from_quaternion(q):
        angles = tr.euler_from_quaternion(q, self.eulerConvention)
        return EulerAngles(angles[0], angles[1], angles[2])

    def to_quaternion(self):
        return Quaternion(tr.quaternion_from_euler(roll,
                                                   pitch,
                                                   yaw,
                                                   self.eulerConvention))

def create_folder(path):

    if os.path.exists(path):
        raise Exception("Failed to create folder : \"" + path
                        + "\" already exists !")
    os.mkdir(path)

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

# def get_pcd_header(filename):
# 
#     res = ""
#     pcdFile = io.open(filename, "rb")
#     fileBuffer = io.BufferedReader(pcdFile, buffer_size=1)
#     # textReader = io.TextIOWrapper(fileBuffer, encoding="ascii")
#     textReader = io.TextIOWrapper(pcdFile, encoding="ascii", errors='backslashreplace')
#    
#     for i in range(400):
#         try:
#             res += textReader.read(1)
#         except Exception as e:
#             print(e) 
#             break
#     
#     return res
 
