import numpy as np
from pyquaternion import Quaternion

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

