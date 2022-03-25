import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


data1 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new1_f.csv',delimiter=",",dtype=float)
data1 = np.delete(data1,0,0)
x1 = data1[:, 1]
y1 = data1[:, 2]
z1 = data1[:, 3]

data2 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new2_f.csv',delimiter=",",dtype=float)
data2 = np.delete(data2,0,0)
x2 = data2[:, 1]
y2 = data2[:, 2]
z2 = data2[:, 3]

data3 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new3_f.csv',delimiter=",",dtype=float)
data3 = np.delete(data3,0,0)
x3 = data3[:, 1]
y3 = data3[:, 2]
z3 = data3[:, 3]

cost1 = genfromtxt('/home/zxl5344/test/src/alei/robotdata/data2pdf_KL1.csv',delimiter=",",dtype=float)
cost1 = np.delete(cost1,0,0)
row1 = cost1[:, 0]
KL1 = cost1[:, 1]

cost2 = genfromtxt('/home/zxl5344/test/src/alei/robotdata/data2pdf_KL2.csv',delimiter=",",dtype=float)
cost2 = np.delete(cost2,0,0)
row2 = cost2[:, 0]
KL2 = cost2[:, 1]

cost3 = genfromtxt('/home/zxl5344/test/src/alei/robotdata/data2pdf_KL3.csv',delimiter=",",dtype=float)
cost3 = np.delete(cost3,0,0)
row3 = cost3[:, 0]
KL3 = cost3[:, 1]


fig = plt.figure()
ax1 = plt.axes(projection='3d')
plt.title("traj",fontsize=30)
plt.xlabel("x",fontsize=30)
plt.ylabel("y",fontsize=30)
ax1.zaxis.set_rotate_label(False) 
ax1.set_zlabel('z', fontsize=30, rotation = 0)
ax1.plot3D(x1,y1,z1, 'r')
#ax1.plot3D(x2,y2,z2, 'b')
#ax1.plot3D(x3,y3,z3, 'y')


plt.figure()
plt.plot(row1,KL1, 'r')
plt.title("cost of demo 1");
plt.xlabel("time window")
plt.ylabel("KL_cost")

plt.figure()
plt.plot(row2,KL2, 'b')
plt.title("cost of demo 2");

plt.figure()
plt.plot(row3,KL3, 'y')
plt.title("cost of demo 3");

plt.show()

