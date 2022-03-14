import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

data=genfromtxt('/home/zxl5344/test/src/alei/robotdata/data2pdftest.csv',delimiter=",",dtype=float)
data = np.delete(data,0,0)
time = data[:,0]
x = data[:, 1]
y = data[:, 2]
z = data[:, 3]
wx = data[:, 4]
wy = data[:, 5]
wz = data[:, 6]
w = data[:, 7]

data1=genfromtxt('/home/zxl5344/test/src/alei/Sweeping data/ArucoPositionSample1.csv',delimiter=",",dtype=float)
data1 = np.delete(data1,0,0)
x1 = data1[:, 1]
y1 = data1[:, 2]
z1 = data1[:, 3]

data2=genfromtxt('/home/zxl5344/test/src/alei/Sweeping data/ArucoPositionSample2.csv',delimiter=",",dtype=float)
data2 = np.delete(data2,0,0)
x2 = data2[:, 1]
y2 = data2[:, 2]
z2 = data2[:, 3]

data3=genfromtxt('/home/zxl5344/test/src/alei/Sweeping data/ArucoPositionSample3.csv',delimiter=",",dtype=float)
data3 = np.delete(data3,0,0)
x3 = data3[:, 1]
y3 = data3[:, 2]
z3 = data3[:, 3]

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x,y,z, 'r')
"""
ax.plot3D(x1,y1,z1, 'b')
ax.plot3D(x2,y2,z2, 'b')
ax.plot3D(x3,y3,z3, 'b')
"""

ax.plot3D([0],[0],[0], 'b',markersize=20)

plt.show()

