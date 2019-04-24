#! /usr/bin/python3

from Metadata import Metadata
from ImageSynchronizer import ImageSynchronizer
import numpy as np
import matplotlib.pyplot as plt
from  matplotlib.patches import Ellipse

dataRootDir = '/local/infuse/data-test/minnie/log_data_acquisition_2018_12_05_14_12_30/raw_data/';
# dataRootDir = '/media/M3-data1/M3-mission/minnie/log_data_acquisition_2018_12_02_13_42_55/raw_data/'

tokamak = Metadata()
tokamak.parse_metadata(dataRootDir + 'tokamak/dataformat.txt', dataRootDir + 'tokamak/tokamak.txt')
x_tokamak = tokamak.get_nparray('x')
y_tokamak = tokamak.get_nparray('y')
Tok = np.vstack((x_tokamak.transpose(), y_tokamak.transpose()))
time_tokamak = tokamak.get_nparray('child_time')
t0_tok = time_tokamak[0]

odo = Metadata()
odo.parse_metadata(dataRootDir + 'odometry/dataformat.txt', dataRootDir + 'odometry/odometry.txt')
x_odo = odo.get_nparray('x')
y_odo = odo.get_nparray('y')
Odo = np.vstack((x_odo.transpose(), y_odo.transpose()))
time_odo = odo.get_nparray('child_time')
t0_odo = time_odo[0]

index = 0
while abs(time_odo[index] - t0_tok) > 25000:
    index += 1
Odo = Odo[:,index:]
print([Odo.shape[1], Tok.shape[1]])
index = min([Odo.shape[1], Tok.shape[1]])
Odo = Odo[:,:index]
Tok = Tok[:,:index]

R = Tok @ Odo.transpose() @ np.linalg.inv(Odo @ Odo.transpose())
U,S,Vh = np.linalg.svd(R)
R = U @ Vh
Odo = R @ Odo
x_odo = Odo[0,:].transpose()
y_odo = Odo[1,:].transpose()

x_odo += x_tokamak[0] - x_odo[0]
y_odo += y_tokamak[0] - y_odo[0]

fig, axes = plt.subplots(1,1, sharex=False, sharey=False)
axes.plot(x_tokamak, y_tokamak, label="tokamak")
axes.plot(x_odo, y_odo, label="odo")
axes.set_xlabel("East (m)")
axes.set_ylabel("North (m)")
axes.legend(loc="upper right")
axes.grid()
plt.axis('equal')

plt.show(block=False)

