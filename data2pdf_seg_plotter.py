import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.pyplot import cm


data1 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new2.csv',delimiter=",",dtype=float)
x1 = data1[:, 1]
y1 = data1[:, 2]
z1 = data1[:, 3]

seg1 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new2_seg.csv',delimiter=",",dtype=float)
s1 = seg1

fig = plt.figure()
ax1 = plt.axes(projection='3d')
color = iter(cm.rainbow(np.linspace(0, 1, len(s1))))
for i in range(1,len(s1)):
	prev = int(s1[i-1])
	curr = int(s1[i])
	new_x1 = x1[prev:curr]
	new_y1 = y1[prev:curr]
	new_z1 = z1[prev:curr]
	c = next(color)
	ax1.plot3D(new_x1,new_y1,new_z1, c=c)
	plt.pause(2)
	
plt.show()

