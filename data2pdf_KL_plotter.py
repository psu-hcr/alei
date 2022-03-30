import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

"""
data1 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_screw4_nonstop.csv',delimiter=",",dtype=float)

x1_seg = data1[10:40, 1]	
y1_seg = data1[10:40, 2]
z1_seg = data1[10:40, 3]

x1_seg = data1[50:75, 1]	
y1_seg = data1[50:75, 2]
z1_seg = data1[50:75, 3]

x1_seg = data1[75:95, 1]	
y1_seg = data1[75:95, 2]
z1_seg = data1[75:95, 3]

x1_seg = data1[95:110, 1]
y1_seg = data1[95:110, 2]
z1_seg = data1[95:110, 3]

x1_seg = data1[110:120, 1]
y1_seg = data1[110:120, 2]
z1_seg = data1[110:120, 3]

x1_seg = data1[120:135, 1]
y1_seg = data1[120:135, 2]
z1_seg = data1[120:135, 3]

x1_seg = data1[135:160, 1]
y1_seg = data1[135:160, 2]
z1_seg = data1[135:160, 3]

x1_seg = data1[160:175, 1]
y1_seg = data1[160:175, 2]
z1_seg = data1[160:175, 3]

x1_seg = data1[175:185, 1]
y1_seg = data1[175:185, 2]
z1_seg = data1[175:185, 3]
"""
"""
x1 = data1[:, 1]
y1 = data1[:, 2]
z1 = data1[:, 3]

data2 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_screw5_nonstop.csv',delimiter=",",dtype=float)
x2 = data2[:, 1]
y2 = data2[:, 2]
z2 = data2[:, 3]

data3 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_screw6_nonstop.csv',delimiter=",",dtype=float)
x3 = data3[:, 1]
y3 = data3[:, 2]
z3 = data3[:, 3]

fig = plt.figure()
ax1 = plt.axes(projection='3d')
plt.title("traj",fontsize=30)
plt.xlabel("x",fontsize=30)
plt.ylabel("y",fontsize=30)
ax1.zaxis.set_rotate_label(False) 
ax1.set_zlabel('z', fontsize=30, rotation = 0)
ax1.plot3D(x1,y1,z1, 'r')
#ax1.plot3D(x1_seg,y1_seg,z1_seg, 'g', linewidth=10)
ax1.plot3D(x2,y2,z2, 'b')
ax1.plot3D(x3,y3,z3, 'y')
"""


cost1 = genfromtxt('/home/zxl5344/test/src/alei/robotdata/data2pdf_KL1.csv',delimiter=",",dtype=float)
cost1 = np.delete(cost1,0,0)
row1 = cost1[:, 0]
KL1 = cost1[:, 1]

plt.figure()
plt.plot(row1,KL1, 'r')
plt.title("cost of demo 1");
plt.xlabel("time window")
plt.ylabel("KL_cost")

"""
cost2 = genfromtxt('/home/zxl5344/test/src/alei/robotdata/data2pdf_KL2.csv',delimiter=",",dtype=float)
cost2 = np.delete(cost2,0,0)
row2 = cost2[:, 0]
KL2 = cost2[:, 1]

plt.figure()
plt.plot(row2,KL2, 'b')
plt.title("cost of demo 2");


cost3 = genfromtxt('/home/zxl5344/test/src/alei/robotdata/data2pdf_KL3.csv',delimiter=",",dtype=float)
cost3 = np.delete(cost3,0,0)
row3 = cost3[:, 0]
KL3 = cost3[:, 1]

plt.figure()
plt.plot(row3,KL3, 'y')
plt.title("cost of demo 3");
"""

dataxyz = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_screw5_nonstop.csv',delimiter=",",dtype=float)
strat = 1
end = int(row1[-1])
x_align = dataxyz[strat:end, 1]
y_align = dataxyz[strat:end, 2]
z_align = dataxyz[strat:end, 3]

fig2, axs2 = plt.subplots(3, 1,sharex=True)
axs2[0].plot(row1,x_align, 'r')
axs2[0].set_ylabel("x")
axs2[1].plot(row1,y_align, 'y')
axs2[1].set_ylabel("y")
axs2[2].plot(row1,z_align, 'b')
axs2[2].set_ylabel("z")
axs2[2].set_xlabel("t")



plt.show()

