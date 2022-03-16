import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


data1 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample1.csv',delimiter=",",dtype=float)
x1 = data1[:, 1]
y1 = data1[:, 2]
z1 = data1[:, 3]


data2 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample2.csv',delimiter=",",dtype=float)
x2 = data2[:, 1]
y2 = data2[:, 2]
z2 = data2[:, 3]

data3 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample3.csv',delimiter=",",dtype=float)
x3 = data3[:, 1]
y3 = data3[:, 2]
z3 = data3[:, 3]


cost = genfromtxt('/home/zxl5344/test/src/alei/robotdata/data2pdf_KL.csv',delimiter=",",dtype=float)
cost = np.delete(cost,0,0)
row = cost[:, 0]
KL = cost[:, 1]

"""
fig = plt.figure()
ax1 = plt.axes(projection='3d')
ax1.plot3D(x1,y1,z1, 'b')
ax1.plot3D(x2,y2,z2, 'r')
ax1.plot3D(x3,y3,z3, 'y')
"""
plt.figure()
plt.plot(row,KL, 'r')

plt.show()

