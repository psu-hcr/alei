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

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x,y,z, 'r')
plt.show()

